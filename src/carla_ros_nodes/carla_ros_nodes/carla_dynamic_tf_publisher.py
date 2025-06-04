import math

import carla
import rclpy
import transforms3d
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class CarlaDynamicTFBroadcaster(Node):
    def __init__(self):
        super().__init__("carla_dynamic_tf_broadcaster")
        self.broadcaster = TransformBroadcaster(self)
        self.get_logger().info(
            "CarlaDynamicTFBroadcaster node started, publishing dynamic TF."
        )

        self.carla_client = None
        self.world = None
        self.vehicle = None
        self.setup_carla()

        self.timer = self.create_timer(
            0.05, self.publish_vehicle_transform
        )  # 20 Hz NOT CORRECT ADJUST

    def setup_carla(self):
        try:
            self.carla_client = carla.Client("localhost", 2000)
            self.carla_client.set_timeout(10.0)
            self.world = self.carla_client.get_world()

            self.vehicle = None
            for actor in self.world.get_actors().filter("vehicle.*"):
                if (
                    "role_name" in actor.attributes
                    and actor.attributes["role_name"] == "ego_vehicle"
                ):
                    self.vehicle = actor
                    self.get_logger().info(
                        f"Found ego vehicle for dynamic TF: {self.vehicle.id}"
                    )
                    break
            if not self.vehicle:
                self.get_logger().error(
                    "Ego vehicle not found for dynamic TF. Please spawn one."
                )
                self.carla_client = None

        except Exception as e:
            self.get_logger().error(f"Error setting up CARLA for dynamic TF: {e}")
            self.carla_client = None

    def publish_vehicle_transform(self):
        if not self.vehicle:
            return

        carla_transform = self.vehicle.get_transform()
        location = carla_transform.location
        rotation = carla_transform.rotation  # Pitch, Yaw, Roll

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = location.x
        t.transform.translation.y = location.y
        t.transform.translation.z = location.z

        # Orientation (CARLA rotation to quaternion)
        # CARLA uses pitch, yaw, roll in degrees
        # ROS uses roll, pitch, yaw in radians
        # +X forward, +Y left, +Z up

        roll = math.radians(rotation.roll)
        pitch = math.radians(rotation.pitch)
        yaw = math.radians(rotation.yaw)

        quat = transforms3d.euler.euler2quat(roll, pitch, yaw)

        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    carla_dynamic_tf_broadcaster = CarlaDynamicTFBroadcaster()
    try:
        rclpy.spin(carla_dynamic_tf_broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        carla_dynamic_tf_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
