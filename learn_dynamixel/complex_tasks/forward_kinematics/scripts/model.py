import matplotlib.pyplot as plt
import numpy as np


def wx200_parameters(motor_1_angle, motor_2_angle, motor_4_angle, motor_5_angle, motor_6_angle):
    """
    Generates Denavit Hartenberg parameters for the WidowX 200 robot arm given
    current motor rotation states.

    :param float motor_1_angle: Current rotation of motor 1 
    :param float motor_2_angle: Current rotation of motor (2 + 3) 
    :param float motor_4_angle: Current rotation of motor 4 
    :param float motor_5_angle: Current rotation of motor 5 
    :param float motor_6_angle: Current rotation of motor 6 
    :returns: WidowX 200 DH parameters
    :rtype: numpy.ndarray
    """
    parameters = np.array([
        # alpha i - 1, a i - 1, d i, theta i
        [0.0, 0.0, 0.0, motor_1_angle],
        [np.pi / 2, 0.0, 0.0, motor_2_angle],
        [0.0, 206.16, 0.0, motor_4_angle],
        [0.0, 200.0, 0.0, motor_5_angle],
        [np.pi / 2, 0.0, 0.0, motor_6_angle],
    ])

    return np.copy(parameters)


def dh_transform(alpha, a, d, theta):
    """
    Generates transformation matrices using Denavit Hartenberg parameters as
    defined by Craig's Introduction to Robotics (2005)

    :param float alpha: \\alpha_{i-1}: Angle from \\hat{Z}_{i-1} to \\hat{Z}_{i} measured about \\hat{X}_{i-1}
    :param float a: a_{i-1}: Distance from \\hat{Z}_{i-1} to \\hat{Z}_{i} measured along \\hat{X}_{i-1}
    :param float d: d_{i}: Distance from \\hat{X}_{i-1} to \\hat{X}_{i} measured along \\hat{Z}_{i}
    :param float theta: \\theta_{i}: Angle from \\hat{X}_{i-1} to \\hat{X}_{i} measured about \\hat{Z}_{i}
    :returns: DH Transform from frame i-1 to frame i
    :rtype: numpy.ndarray
    """

    transform = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])

    return np.copy(transform)


def plot_model(frames):
    """
    Plot a three dimensional model of a robotic manipulator using Denavit
    Hartenbert transformations.

    :param list frames: List of transformations between joint coordinate spaces
    """

    # Save the origin of each frame with respect to the world frame
    x = [0]
    y = [0]
    z = [0]

    '''
    Expected matrix form
    [r_11, r_12, r_13, p_x]
    [r_21, r_22, r_23, p_y]
    [r_31, r_32, r_33, p_z]
    [  0 ,   0 ,   0 ,  1 ]
    '''
    for frame in frames:
        x.append(frame[0][3])
        y.append(frame[1][3])
        z.append(frame[2][3])

    # for i in range(len(x)):
    #     print(x[i], y[i], z[i])

    # Create new figure and include a 3d plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Labeling
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Plot each frame origin as a point linked by lines
    ax.plot(x, y, z, marker='.', markersize=20)

    # Arbitrary axis marks
    ax.set_xticks([-500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500])
    ax.set_yticks([-500, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500])
    ax.set_zticks([-450, -400, -300, -200, -100, 0, 100, 200, 300, 400, 500])

    plt.show()


def main():
    # Generate WidowX 200 Denavit Hartenberg parameters
    p = wx200_parameters(0.0, 0.0 + np.cos(50/200), 0.0 - np.cos(50/200), 0.0, 0.0)

    # Collect transformation matrices from world to each joint
    T = [np.array([])] * p.shape[0]
    for i in range(p.shape[0]):
        if i == 0:
            T[i] = dh_transform(*[p[i][j] for j in range(p.shape[1])])
        else:
            # Each matrix after the first builds off of the previous matrices
            T[i] = T[i - 1] @ dh_transform(*[p[i][j] for j in range(p.shape[1])])
    
    # Plot model of arm
    plot_model(T)


if __name__ == "__main__":
    main()

