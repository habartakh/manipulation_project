#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include <object_detection/msg/detected_objects.hpp>
using std::placeholders::_1;

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // Initialize a subscriber node to get the detected object position
    object_pose_node =
        rclcpp::Node::make_shared("object_pose_node", node_options);
    object_pose_sub = object_pose_node->create_subscription<DetectedObjects>(
        "object_detected", 10,
        std::bind(&PickAndPlaceTrajectory::topic_callback, this, _1));
    object_pose_future_ = object_pose_promise_.get_future();
    executor_.add_node(object_pose_node);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() {
      //   executor_.add_node(object_pose_node);
      //   executor_.add_node(move_group_node_);

      this->executor_.spin();
    }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    initial_state_robot_ = move_group_robot_->getCurrentState(10);
    initial_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    initial_state_gripper_ = move_group_gripper_->getCurrentState(10);
    initial_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // Store the initial joints positions to use later to make the robot return
    // to its initial position at the end of the pick and place sequence
    initial_joint_group_positions_robot_ = joint_group_positions_robot_;
    initial_joint_group_positions_gripper_ = joint_group_positions_gripper_;

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlaceTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place Trajectory...");

    // wait till we get detected object position from the topic
    RCLCPP_INFO(LOGGER, "Waiting for object pose...");

    auto status = object_pose_future_.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(LOGGER, "Timeout while waiting for object pose!");
      return;
    }

    // First, close the gripper
    set_gripper_value(+0.700);

    // Then move to the pregrasp position
    go_to_pregrasp_position();

    // wait one second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Open the gripper to prepare for grasping the object
    set_gripper_value(+0.200);

    // wait one second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Move down even closer to the object
    approach_object();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Close the gripper to grasp the object
    grasp_object();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Move back up while carrying the object
    retreat();

    // Turn the shoulder joint to go to the loading position
    turn_shoulder_180();

    // And release the object
    set_gripper_value(+0.200);

    // After finishing the task, return to the original position
    return_to_initial_position();

    RCLCPP_INFO(LOGGER, "Pick And Place Trajectory Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;
  using DetectedObjects = object_detection::msg::DetectedObjects;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // subscriber to get the detected object position
  rclcpp::Node::SharedPtr object_pose_node;
  rclcpp::Subscription<DetectedObjects>::SharedPtr object_pose_sub;

  // Get the object detected position asynchronously with future object
  std::promise<DetectedObjects> object_pose_promise_;
  std::future<DetectedObjects> object_pose_future_;

  // declare Single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot and gripper
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  std::vector<double> initial_joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  RobotStatePtr initial_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  std::vector<double> joint_group_positions_gripper_;
  std::vector<double> initial_joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  RobotStatePtr initial_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  // initial position of the robot in reference to base_link of the arm
  geometry_msgs::msg::Point object_initial_position;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  // setup the goal pose target
  void go_to_pregrasp_position() {

    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position...");
    RCLCPP_INFO(LOGGER,
                "In pregrasp function, here is the Detected object pose is : "
                "%f , %f , %f",
                object_initial_position.x, object_initial_position.y,
                object_initial_position.z);
    // Place the gripper at z_offset from the object
    double z_offset = 0.22;
    double y_offset = -0.01;
    double x_offset = 0.0105;
    setup_goal_pose_target(object_initial_position.x + x_offset,
                           object_initial_position.y + y_offset,
                           object_initial_position.z + z_offset, -1.000, 0.000,
                           0.000, 0.000);

    // plan and execute the trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  void set_gripper_value(double gripper_joint_value) {
    // open the gripper
    RCLCPP_INFO(LOGGER, "Set Gripper joint value...");
    // setup the gripper target by pose name
    setup_joint_value_gripper(gripper_joint_value);
    // plan and execute the trajectory
    plan_trajectory_gripper();
    execute_trajectory_gripper();
  }

  void approach_object() {
    RCLCPP_INFO(LOGGER, "Approaching...");
    // setup the cartesian target
    setup_waypoints_target(+0.000, +0.000, -0.085);
    // plan and execute the trajectory
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
  }

  void grasp_object() {

    RCLCPP_INFO(LOGGER, "Gradually closing the gripper...");

    double gripper_value = +0.600;

    // Then, close gradually the gripper
    // If the movement is too sudden, the object falls
    for (int i = 0; i < 3; ++i) {
      set_gripper_value(gripper_value);
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      gripper_value += 0.010;
    }

    // Finetune the gripper joint values even further
    for (int i = 0; i < 3; ++i) {

      gripper_value += 0.001;
      std::cout << "####################################" << std::endl;
      std::cout << "NOW IS THE ST TIME :" << gripper_value << std::endl;
      std::cout << "####################################" << std::endl;

      set_gripper_value(gripper_value);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  void retreat() {
    RCLCPP_INFO(LOGGER, "Retreating...");
    // setup the cartesian target
    setup_waypoints_target(+0.000, +0.000, +0.070);
    // plan and execute the trajectory
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
  }

  void turn_shoulder_180() {
    RCLCPP_INFO(LOGGER, "Turning the shoulder joint by 180° degrees...");
    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    // setup the joint value target
    setup_joint_value_target(
        +3.14, joint_group_positions_robot_[1], joint_group_positions_robot_[2],
        joint_group_positions_robot_[3], joint_group_positions_robot_[4],
        joint_group_positions_robot_[5]);
    // plan and execute the trajectory
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
  }

  void return_to_initial_position() {

    RCLCPP_INFO(LOGGER, "Returning to the initial pose...");

    // Return to the initial state of the robot
    move_group_robot_->setJointValueTarget(
        initial_joint_group_positions_robot_);

    plan_trajectory_kinematics();
    execute_trajectory_kinematics();

    // Then set the gripper to its initial position
    move_group_gripper_->setJointValueTarget(
        initial_joint_group_positions_gripper_);
    plan_trajectory_gripper();
    execute_trajectory_gripper();
  }

  // Detected object position subscriber callback
  // returns the object's position in reference to the base_link of the arm
  void topic_callback(const DetectedObjects::SharedPtr msg) {
    RCLCPP_INFO(LOGGER, "Looking for any detected objects...");

    // If an object or more were detected
    // Make sure that the object is as tall as the cube to be grasped
    if (msg != nullptr && msg->thickness > 0.04) {

      object_initial_position.x = msg->position.x;
      object_initial_position.y = msg->position.y;
      object_initial_position.z = msg->position.z;

      RCLCPP_INFO(
          LOGGER,
          "In topic callback, the Detected object pose is : %f , %f , %f",
          object_initial_position.x, object_initial_position.y,
          object_initial_position.z);

      object_pose_promise_.set_value(*msg); // Fulfilled the promise

      // Then unsubscribe from the topic
      object_pose_sub.reset();
    }
  }

}; // class PickAndPlaceTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place_trajectory");

  // instantiate class
  PickAndPlaceTrajectory pick_and_place_trajectory_node(base_node);

  // execute trajectory plan
  pick_and_place_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}