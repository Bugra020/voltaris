import sys  # Import sys for better error handling
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

        self.is_carla_setup_successful = False
        self.setup_carla()

        if not self.is_carla_setup_successful:
            self.get_logger().error(
                "CARLA setup failed. Shutting down lidar_publisher."
            )
            rclpy.shutdown()

    def setup_carla(self):
        self.get_logger().info("Attempting to connect to CARLA server...")
        try:
            self.carla_client = carla.Client("localhost", 2000)
            self.carla_client.set_timeout(10.0)  # Increased timeout for robustness
            self.get_logger().info("Connected to CARLA client.")

            self.world = self.carla_client.get_world()
            self.get_logger().info(
                f"Retrieved CARLA world: {self.world.get_map().name}"
            )

            self.vehicle = None
            for actor in self.world.get_actors().filter("vehicle.*"):
                if (
                    "role_name" in actor.attributes
                    and actor.attributes["role_name"] == "ego_vehicle"
                ):
                    self.vehicle = actor
                    self.get_logger().info(
                        f"Found existing ego vehicle (ID: {self.vehicle.id}, Type: {self.vehicle.type_id})"
                    )
                    break

            if not self.vehicle:
                self.get_logger().info(
                    "No ego vehicle found with role_name 'ego_vehicle', attempting to spawn a new one..."
                )
                blueprint_library = self.world.get_blueprint_library()

                vehicle_bp = blueprint_library.filter("vehicle.audi.a2")[0]
                vehicle_bp.set_attribute("role_name", "ego_vehicle")

                spawn_points = self.world.get_map().get_spawn_points()
                if not spawn_points:
                    self.get_logger().error(
                        "No spawn points available in CARLA map. Cannot spawn vehicle."
                    )
                    self.is_carla_setup_successful = False
                    return

                spawn_point = spawn_points[0]

                self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                self.get_logger().info(
                    f"Spawned new ego vehicle (ID: {self.vehicle.id}, Type: {self.vehicle.type_id}) at {spawn_point.location}"
                )
                # self.vehicle.set_autopilot(True)

            self.get_logger().info("Attempting to spawn LIDAR sensor...")
            lidar_bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
            lidar_bp.set_attribute("channels", "32")
            lidar_bp.set_attribute("range", "50.0")
            lidar_bp.set_attribute("points_per_second", "100000")
            lidar_bp.set_attribute("rotation_frequency", "20")

            lidar_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
            self.lidar_sensor = self.world.spawn_actor(
                lidar_bp, lidar_transform, attach_to=self.vehicle
            )
            self.get_logger().info("LIDAR sensor spawned. Starting to listen for data.")
            self.lidar_sensor.listen(lambda data: self.lidar_callback(data))
            self.get_logger().info("LIDAR sensor is listening. Expecting data soon.")

            self.is_carla_setup_successful = True

        except Exception as e:
            self.get_logger().error(
                f"Error during CARLA setup: {e}", throttle_duration_sec=5
            )
            self.is_carla_setup_successful = False
            if self.carla_client:
                self.carla_client = None
            if self.world:
                if self.lidar_sensor and self.lidar_sensor.is_alive:
                    self.lidar_sensor.destroy()
                if self.vehicle and self.vehicle.is_alive:
                    self.vehicle.destroy()

            self.get_logger().error(f"Detailed traceback: {sys.exc_info()[2]}")

    def lidar_callback(self, carla_lidar_data):
        self.get_logger().info(
            f"LIDAR callback triggered! Data point count: {len(carla_lidar_data.raw_data) / 16}"
        )  # Be careful, this can spam logs

        # if not self.publisher_.get_subscription_count():
        # self.get_logger().info("No subscribers to /velodyne_points, skipping publication.") # Optional: log when skipping
        #    return

        points = np.frombuffer(carla_lidar_data.raw_data, dtype=np.dtype("f4"))

        if points.size == 0 or points.shape[0] % 4 != 0:
            self.get_logger().warn(
                f"Received empty or malformed LIDAR data. Size: {points.size}",
                throttle_duration_sec=1,
            )
            return

        points = np.reshape(points, (int(points.shape[0] / 4), 4))

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
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.data = points.astype(np.float32).tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info(
            "Destroying CarlaLidarPublisher node. Cleaning up CARLA actors..."
        )
        if self.lidar_sensor and self.lidar_sensor.is_alive:
            self.lidar_sensor.destroy()
            self.get_logger().info("LIDAR sensor destroyed.")
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.get_logger().info("Ego vehicle destroyed.")

        time.sleep(0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    carla_lidar_publisher = CarlaLidarPublisher()

    if not carla_lidar_publisher.is_carla_setup_successful:
        carla_lidar_publisher.get_logger().error(
            "Node initialization failed due to CARLA setup issues."
        )
        sys.exit(1)

    try:
        rclpy.spin(carla_lidar_publisher)
    except KeyboardInterrupt:
        carla_lidar_publisher.get_logger().info("Node stopped by KeyboardInterrupt.")
    except Exception as e:
        carla_lidar_publisher.get_logger().error(f"Unhandled exception in spin: {e}")
    finally:
        carla_lidar_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
