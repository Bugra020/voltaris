import time

import carla
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CarlaLidarPublisher(Node):
    def __init__(self):
        super().__init__("carla_lidar_publisher")
        self.publisher_ = self.create_publisher(PointCloud2, "/velodyne_points", 10)
        self.get_logger().info(
            "CarlaLidarPublisher node started, publishing to /velodyne_points"
        )

        self.carla_client = None
        self.world = None
        self.lidar_sensor = None
        self.vehicle = None
        self.setup_carla()

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
                        f"Found existing ego vehicle: {self.vehicle.id}"
                    )
                    break

            # If no ego vehicle found, spawn one
            if not self.vehicle:
                self.get_logger().info(
                    "No ego vehicle found, attempting to spawn one..."
                )
                blueprint_library = self.world.get_blueprint_library()

                vehicle_bp = blueprint_library.filter("vehicle.*")[0]

                vehicle_bp.set_attribute("role_name", "ego_vehicle")

                spawn_points = self.world.get_map().get_spawn_points()
                if not spawn_points:
                    self.get_logger().error(
                        "No spawn points available in CARLA map. Cannot spawn vehicle."
                    )
                    return

                spawn_point = spawn_points[0]

                self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                self.get_logger().info(
                    f"Spawned new ego vehicle: {self.vehicle.id} at {spawn_point.location}"
                )

            lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("channels", "32")
            lidar_bp.set_attribute("range", "50.0")
            lidar_bp.set_attribute("points_per_second", "600000")
            lidar_bp.set_attribute("rotation_frequency", "10")
            lidar_bp.set_attribute("dropoff_general_rate", "0.0")
            lidar_bp.set_attribute("dropoff_intensity_limit", "0.0")
            lidar_bp.set_attribute("dropoff_zero_intensity", "0.0")

            lidar_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
            self.lidar_sensor = self.world.spawn_actor(
                lidar_bp, lidar_transform, attach_to=self.vehicle
            )
            self.lidar_sensor.listen(lambda data: self.lidar_callback(data))
            self.get_logger().info("LIDAR sensor spawned and listening.")

        except Exception as e:
            self.get_logger().error(f"Error setting up CARLA: {e}")
            self.carla_client = None

    def lidar_callback(self, carla_lidar_data):
        # if not self.publisher_.get_subscription_count():
        #    return

        # carla lidar data contains (x, y, z, intensity)
        points = np.frombuffer(carla_lidar_data.raw_data, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))

        # create pointCloud2 msg
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "velodyne"

        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = False

        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        msg.point_step = 16  # 4 floats * 4 bytes/float
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.data = points.astype(np.float32).tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.lidar_sensor:
            self.lidar_sensor.destroy()
            self.get_logger().info("LIDAR sensor destroyed.")
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.get_logger().info("LIdar sensor destroyed.")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    carla_lidar_publisher = CarlaLidarPublisher()
    try:
        rclpy.spin(carla_lidar_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        carla_lidar_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
