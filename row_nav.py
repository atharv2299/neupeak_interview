import numpy as np
import matplotlib.pyplot as plt

"""
    Test Description:

    [TASK 1]
    Create an algorithm that takes in the following:
        row_pointcloud : scanned pointcloud (3D numpy array) for the row from depth camera

    And outputs the following:
        angular_rate : rate in deg/s the robot should turn at such that it will centre itself in the row
	
	The goal is to genereate a target angular rate that the robot uses to turn. Positive rate means robot
	will turn right, negative rate means robot will turn left. Assume the robot moves forward at some constant velocity
	set by the user. Your algorithm then generates angular rates for the robot while it moves through the row. The ideal angular 
	rate will make it so that the robot is always centred in the row. When the robot is perfectly centred the angular rate should be 0
	since we just need the robot to go straight.

    [TASK 2]
    Implement a ROS Node that will output correction anglular rate for the robot. Publish cmd_vel to a twist
	ROS topic

    [TASK 3]
    unit-test: Write a unit test script using standard Python unittest library

    [OPTIONAL BONUS 1]
    end_of_row: Boolean parameter that is True if end of row is detected

    


    Evaluation Criteria: Method runtime. Lower is better.
"""


def plot_row_pointcloud(pointcloud):
    """
    pointcloud: [numpy array]: pointcloud to be displayed
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.view_init(azim=0, elev=-180)
    ax.scatter(pointcloud[:, 2], pointcloud[:, 0], pointcloud[:, 1])
    plt.show()


def read_pc_from_file(file):
    """
    file: [string]: npz file to be processed
    return: pointcloud in numpy array
    """
    row_np_array = np.load(file)
    row_pointcloud = row_np_array["arr_0"]
    return row_pointcloud


def pcloud_info(pc):
    """
    This is a helper function I've written just to help me understand the properties
    of any pointcloud I pass to it

    pc: a pointcloud in numpy array form
    """

    # Based on the plotting provided x  is col 2, y is col 0,  and z is col 1
    x = pc[:, 2]
    y = pc[:, 0]
    z = pc[:, 1]

    print(f"Minimum x: {min(x)} \nMaximum x: {max(x)}")
    print(f"Minimum y: {min(y)} \nMaximum y: {max(y)}")
    print(f"Minimum z: {min(z)} \nMaximum z: {max(z)}")


def reduce_pc(pointcloud):
    """
    This is a helper function to reduce the pointcloud to more relevant info.

    pointcloud: a pointcloud in numpy array form
    return: a reduced pointcloud in numpy array form

    """
    # The lower bound cuts out the floor which isn't relevant to the problem, and
    # the upper bound is set somewhat arbitrarily to cut down on the number of points
    reduced_pc = pointcloud[pointcloud[:, 1] < -0.1]
    reduced_pc = reduced_pc[reduced_pc[:, 1] >= -0.6]
    return reduced_pc


def get_rate(pointcloud):
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

    paired = np.concatenate((norms, angles), axis=0)
    # Solution 1: Base on number of points falling in each half
    ANGULAR_VELOCITY = 20  # max angular velocity in deg/s
    left = norms[angles < 0]
    right = norms[angles > 0]

    angular_rate = ANGULAR_VELOCITY * (left.size - right.size) / norms.size

    return angular_rate


if __name__ == "__main__":
    pointcloud = read_pc_from_file("1.npz")
    pc = reduce_pc(pointcloud)
    # pcloud_info(pc)
    rate = get_rate(pc)
