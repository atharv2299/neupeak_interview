import rclpy
from rclpy.node import Node


from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist


class AngularRate(Node):
    def __init__(self):
        super().__init__("angular rate setter")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pc_sub = self.create_subscription(
            PointCloud, "pointcloud", self.pc_callback, 10
        )
        self.pc_sub

    def pc_callback(self, pointcloud):
        pass


def main():
    rclpy.init()
    angular_rate_node = AngularRate()
    rclpy.spin(angular_rate_node)
    angular_rate_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
