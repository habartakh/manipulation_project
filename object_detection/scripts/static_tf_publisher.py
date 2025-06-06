#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Without this node, the tfs can sometimes stop being published
# thus causing malfunctions during the object detection
class StaticTransformPublisher:
    def __init__(self) -> None:
        self.node = rclpy.create_node('static_transform_publisher_node')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self.node)

    # The tf values are obtained by running :
    # ros2 run tf2_ros tf2_echo base_link wrist_rgbd_camera_depth_optical_frame
    def publish_static_transform(self) -> None:
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.node.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'base_link'
        static_transform_stamped.child_frame_id = 'wrist_rgbd_camera_depth_optical_frame'
        
        static_transform_stamped.transform.translation.x = 0.338  
        static_transform_stamped.transform.translation.y = 0.450  
        static_transform_stamped.transform.translation.z = 0.100  
        static_transform_stamped.transform.rotation.x = 0.000 
        static_transform_stamped.transform.rotation.y = 0.866  
        static_transform_stamped.transform.rotation.z = -0.500  
        static_transform_stamped.transform.rotation.w = 0.000  
        # Publish the static transform
        self.broadcaster.sendTransform(static_transform_stamped)

    def spin(self) -> None:
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

def main(args=None) -> None:
    rclpy.init(args=args)
    static_transform_publisher = StaticTransformPublisher()
    static_transform_publisher.publish_static_transform()
    static_transform_publisher.spin()

if __name__ == '__main__':
    main()