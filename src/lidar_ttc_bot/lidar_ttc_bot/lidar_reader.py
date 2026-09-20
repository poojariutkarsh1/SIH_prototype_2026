import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class LidarReader(Node):

    def __init__(self):
        super().__init__('lidar_reader')

        self.robot_speed = 0.0

        # Subscribe to LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar_ros_plugin/out',
            self.lidar_callback,
            10
        )

        # Subscribe to vehicle speed command
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )


    def cmd_vel_callback(self, msg):

        self.robot_speed = msg.linear.x


    def lidar_callback(self, msg):

        front_ranges = []

        # Check LiDAR points
        for i, distance in enumerate(msg.ranges):

            angle = msg.angle_min + i * msg.angle_increment

            # Look only in front: -15° to +15°
            if -0.26 < angle < 0.26:

                # Ignore infinity values
                if math.isfinite(distance):
                    front_ranges.append(distance)


        if front_ranges:

            closest = min(front_ranges)

            # Avoid division by zero
            if self.robot_speed <= 0:

                print(
                    f"Distance: {closest:.2f} m | "
                    "Vehicle stopped"
                )

                return


            # TTC calculation
            ttc = closest / self.robot_speed


            # SAFE ZONE
            if ttc > 6.0:

                zone = "SAFE"


            # CAUTION ZONE
            elif ttc > 3.0:

                zone = "CAUTION"


            # DANGER ZONE
            else:

                zone = "DANGER"


            print(
                f"Distance: {closest:.2f} m | "
                f"Speed: {self.robot_speed:.2f} m/s | "
                f"TTC: {ttc:.2f} s | "
                f"{zone}"
            )


        else:

            print("No obstacle detected in front")


def main(args=None):

    rclpy.init(args=args)

    node = LidarReader()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()