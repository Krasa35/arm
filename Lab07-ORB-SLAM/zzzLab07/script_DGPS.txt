import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import NavSatFix
import math
from utm import from_latlon


class Coord:
    def __init__(self):
        self.x = float("nan")
        self.y = float("nan")
        self.z = float("nan")

    def update(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.subscription = self.create_subscription(
            NavSatFix,
            'dgps_ublox/fix',
            self.listener_callback,
            10
        )

        self.coord = Coord()
        self.file = open("/arm_ws/task2.txt", "a", encoding="utf-8")
        self.get_logger().info("File opened for writing.")

    def listener_callback(self, msg: NavSatFix):
        utm_vals = from_latlon(msg.latitude, msg.longitude)

        if (msg.status.status == 2) and math.isnan(self.coord.x):
            self.coord.update(utm_vals[0], utm_vals[1], msg.altitude)

        time = Time.from_msg(msg.header.stamp)
        string = (
            f"{time.nanoseconds / 1e9}, "
            f"{utm_vals[0] - self.coord.x}, "
            f"{utm_vals[1] - self.coord.y}, "
            f"{msg.altitude - self.coord.z}, 0, 0, 0, 1\n"
        )

        self.get_logger().info(string)

        if not math.isnan(self.coord.x):
            self.file.write(string)
            self.file.flush()

    def destroy_node(self):
        self.get_logger().info("Shutting down node, closing file...")
        if not self.file.closed:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()