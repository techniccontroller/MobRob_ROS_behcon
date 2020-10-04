import numpy as np

def get_T_world_robot(yaw_ego, x_ego, y_ego):
    """
    Get transformation from world to robot

    :param yaw_ego: yaw angle of robot (orientation)
    :type yaw_ego: float
    :param x_ego: x position of robot in world space
    :type x_ego: float
    :param y_ego: y position of robot in world space
    :type y_ego: float
    :return: transformationmatrix world->robot
    :rtype: numpy.array(4,4)
    """
    T_world_robot = np.array([[np.cos(yaw_ego), -np.sin(yaw_ego), 0.0, x_ego],
                             [np.sin(yaw_ego), np.cos(yaw_ego), 0.0, y_ego],
                             [0.0, 0.0, 1.0, 0.0],
                             [0.0, 0.0, 0.0, 1.0]])
    return T_world_robot


def get_T_robot_laser():
    """
    Get transformation from robot to laser

    :return: transformationmatrix robot->laser
    :rtype: numpy.array(4,4)
    """
    T_robot_laser = np.array([[1.0, 0.0, 0.0, 0.055],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.06],
                             [0.0, 0.0, 0.0, 1.0]])
    return T_robot_laser


def get_T_laser_cluster(x_cls, y_cls):
    """
    Get transformationmatrix from laser to cluster

    :param x_cls: x coordinate of cluster in laser space
    :type x_cls: float
    :param y_cls: y coordinate of cluster in laser space
    :type y_cls: float
    :return: transformationmatrix laser->cluster
    :rtype: numpy.array(4,4)
    """
    return get_T_of_3Dpoint(x_cls, y_cls)


def get_T_of_3Dpoint(x, y, z=0.0):
    """
    Get transformationmatix of simple 3D translation vector

    :param x: x coordinate of 3D vector
    :type x: float
    :param y: x coordinate of 3D vector
    :type y: float
    :param z: x coordinate of 3D vector, defaults to 0.0
    :type z: float, optional
    :return: transfomationmatrix
    :rtype: numpy.array(4,4)
    """
    T_laser_cluster = np.array([[1.0, 0.0, 0.0, x],
                             [0.0, 1.0, 0.0, y],
                             [0.0, 0.0, 1.0, z],
                             [0.0, 0.0, 0.0, 1.0]])
    return T_laser_cluster


def get_T_world_cluster(yaw_ego, x_ego, y_ego, x_cls, y_cls):
    """
    Get transformation from world to laserpoint cluster

    :param yaw_ego: yaw angle of robot (orientation)
    :type yaw_ego: float
    :param x_ego: x position of robot in world space
    :type x_ego: float
    :param y_ego: y position of robot in world space
    :type y_ego: float
    :param x_cls: x coordinate of cluster in laser space
    :type x_cls: float
    :param y_cls: y coordinate of cluster in laser space
    :type y_cls: float
    :return: transfomationmatrix world->cluster
    :rtype: numpy.array(4,4)
    """
    T_world_robot = get_T_world_robot(yaw_ego, x_ego, y_ego)
    T_robot_laser = get_T_robot_laser()
    T_laser_cluster = get_T_laser_cluster(x_cls, y_cls)
    
    T_world_cluster = T_world_robot.dot(T_robot_laser).dot(T_laser_cluster)

    return T_world_cluster


def get_world_coordinate(currentPose, x, y):
    """
    function transforms a point in robot space to world space

    :param currentPose: current pose of robot in world space (x, y, yaw)
    :type currentPose: (float, float, float)
    :param x: x coordinate in robot space
    :type x: float
    :param y: y coordinate in robot space
    :type y: float
    """
    yaw_ego = currentPose[2]
    x_ego = currentPose[0]
    y_ego = currentPose[1]
    T_world_robot = get_T_world_robot(yaw_ego, x_ego, y_ego)

    vector = np.transpose(np.array([x, y, 0, 1]))
    world = T_world_robot.dot(vector)
    
    return (world[0], world[1])

def get_laser_coordinate(x, y):
    """
    function transforms a point in robot space to laser space

    :param x: x coordinate in robot space
    :type x: float
    :param y: y coordinate in robot space
    :type y: float
    """
    T_robot_laser = get_T_robot_laser()
    T_laser_robot = np.linalg.inv(T_robot_laser)

    vector = np.transpose(np.array([x, y, 0, 1]))
    laser = T_laser_robot.dot(vector)
    
    return (laser[0], laser[1])

def get_robot_coordinate(x, y):
    """
    function transforms a point in laser space to robot space

    :param x: x coordinate in laser space
    :type x: float
    :param y: y coordinate in laser space
    :type y: float
    """
    T_robot_laser = get_T_robot_laser()

    vector = np.transpose(np.array([x, y, 0, 1]))
    robot = T_robot_laser.dot(vector)
    
    return (robot[0], robot[1])


def get_robot_corners(currentPose):
    """
    Get corners of robot as coordinates in world space

    :param currentPose: current pose of robot in world space (x, y, yaw)
    :type currentPose: (float, float, float)
    :return: points of all four corners (pt1, pt2, pt3, pt4)
    :rtype: ((float,float),(float,float),(float,float),(float,float))
    """
    # current Pose = T_world_robot
    # points of rect = T_robot_points1/2/3/4
    # points of rect in world = T_world_robot * T_robot_points1/2/3/4

    world_pointLF = get_world_coordinate(currentPose, 0.20, 0.125)
    world_pointRF = get_world_coordinate(currentPose, 0.20, -0.125)
    world_pointRB = get_world_coordinate(currentPose, -0.20, -0.125)
    world_pointLB = get_world_coordinate(currentPose, -0.20, 0.125)

    return (world_pointLF, world_pointRF, world_pointRB, world_pointLB)