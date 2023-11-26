import numpy as np
import rclpy

from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from sensor_msgs.msg import PointCloud


class AngularRate(Node):
    def __init__(self):
        super().__init__("angular rate setter")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pc_sub = self.create_subscription(
            PointCloud, "pointcloud", self.pc_callback, 10
        )
        self.pc_sub

    def pc_callback(self, pointcloud):
        z = self.get_rate(pointcloud)
        # I'm assuming here that publishing the angular velocity to cmd_vel
        # does not affect the linear velocity. Though in a real scenario this
        # would probably be the case as the Twist message contains linear and angular
        # velocity. To remedy either determine the linear velocity in this node and
        #  publish, or subscribe to the topic that is determining it and combine the
        #  linear  and angular rates into a single message somewhere and publish
        ang_vel = Vector3(0, 0, z)
        vel = Twist
        vel.angular = ang_vel

        self.vel_pub.publish(vel)

    def get_rate(self, pointcloud):
        """
        Function to determine the angular rate required from the

        pointcloud: a pointcloud in numpy array form
        return: an angular rate for the robot to turn
        """
        xy_points = pointcloud[:, (2, 0)]
        x = pointcloud[:, 2]
        y = pointcloud[:, 0]

        norms = np.linalg.norm(xy_points, axis=1).reshape(1, -1)
        angles = np.arctan2(y, x).reshape(1, -1)

        ANGULAR_VELOCITY = 20

        left = norms[angles < 0]
        right = norms[angles > 0]
        left_median = np.median(left)
        right_median = np.median(right)

        point_based = left.size - right.size
        median_based = right_median - left_median
        norm_point = point_based / norms.size
        norm_med = median_based / max(right_median, left_median)

        angular_rate = ANGULAR_VELOCITY * (norm_point + norm_med) / 2

        if np.abs(angular_rate) < 3:
            angular_rate = 0
        elif np.abs(angular_rate) > ANGULAR_VELOCITY:
            sign = np.sign(angular_rate)
            angular_rate = sign * ANGULAR_VELOCITY

        return angular_rate


def main():
    rclpy.init()
    angular_rate_node = AngularRate()
    rclpy.spin(angular_rate_node)
    angular_rate_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
