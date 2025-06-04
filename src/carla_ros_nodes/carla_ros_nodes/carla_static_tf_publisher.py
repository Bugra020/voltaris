import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class CarlaStaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("carla_static_tf_broadcaster")
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()
        self.get_logger().info(
            "CarlaStaticTFBroadcaster node started, publishing static TF."
        )

    def publish_static_transforms(self):
        # define the transform from base_link to velodyne
        # these values should match the physical mounting of the LIDAR on the car
        # and the arguments in launch files tf node.
        # arguments=['0','0','0','0','0','0','1','base_link','velodyne']
        # corresponds to x=0, y=0, z=0, roll=0, pitch=0, yaw=0 (quaternion 0,0,0,1 for no rotation)

        t_velodyne = TransformStamped()
        t_velodyne.header.stamp = self.get_clock().now().to_msg()
        t_velodyne.header.frame_id = "base_link"
        t_velodyne.child_frame_id = "velodyne"
        t_velodyne.transform.translation.x = 1.5
        t_velodyne.transform.translation.y = 0.0
        t_velodyne.transform.translation.z = 2.0

        t_velodyne.transform.rotation.x = 0.0
        t_velodyne.transform.rotation.y = 0.0
        t_velodyne.transform.rotation.z = 0.0
        t_velodyne.transform.rotation.w = 1.0

        self.broadcaster.sendTransform(t_velodyne)
        self.get_logger().info(
            f"Published static transform: {t_velodyne.header.frame_id} -> {t_velodyne.child_frame_id}"
        )


def main(args=None):
    rclpy.init(args=args)
    carla_static_tf_broadcaster = CarlaStaticTFBroadcaster()
    try:
        rclpy.spin(carla_static_tf_broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        carla_static_tf_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
