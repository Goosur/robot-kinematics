import sympy as sp
import numpy as np
import matplotlib.pyplot as plt


def fk_transform(alpha, a, d, theta):
    return Matrix([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]
    ])


def sympy_stuff():
    t1, t2, t4, t5, t6 = symbols("theta1,theta2,theta4,theta5,theta6")
    dh = [
        [0, 0, 113.25, t1],
        [pi/2, 0, 0, t2 - pi/2],
        [0, 200, 0, pi/2],
        [0, 50, 0, t4 - pi],
        [0, 200, 0, t5 - pi/2],
        [pi/2, 0, 0, t6],
        [0, 0, 174.15, 0]
    ]

    T = fk_transform(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
    for i in range(1, 7):
        T = T @ fk_transform(dh[i][0], dh[i][1], dh[i][2], dh[i][3])

    T = simplify(T)
    print("\n0 to 7 Transform Matrix")
    pprint(T)

    p = T[0:3, 3]
    roll_pitch_yaw = Matrix([[t6], [t2 + t4 + t5], [t1]])
    roll_pitch = Matrix([[t6], [t2 + t4 + t5]])

    pose = Matrix.vstack(p, roll_pitch)

    J = simplify(pose.jacobian(Matrix([t1, t2, t4, t5, t6])))
    print("\nJacobian Matrix")
    pprint(J)


# [theta1, theta2, theta4, theta5, theta6]
# [  0   ,   1   ,   2   ,   3   ,   4   ]
def jacobian(theta1, theta2, theta4, theta5, theta6):
    return np.array([
        [(-200.0 * sin(theta2) - 50.0 * cos(theta2) + 200.0 * cos(theta2 + theta4) - 174.15 * cos(theta2 + theta4 + theta5)) * sin(theta1),
         (-50.0 * sin(theta2) + 200.0 * sin(theta2 + theta4) - 174.15 * sin(theta2 + theta4 + theta5) + 200.0 * cos(theta2)) * cos(theta1),
         (200.0 * sin(theta2 + theta4) - 174.15 * sin(theta2 + theta4 + theta5)) * cos(theta1),
         -174.15 * sin(theta2 + theta4 + theta5) * cos(theta1), 0],

        [(200.0 * sin(theta2) + 50.0 * cos(theta2) - 200.0 * cos(theta2 + theta4) + 174.15 * cos(theta2 + theta4 + theta5)) * cos(theta1),
         (-50.0 * sin(theta2) + 200.0 * sin(theta2 + theta4) - 174.15 * sin(theta2 + theta4 + theta5) + 200.0 * cos(theta2)) * sin(theta1),
         (200.0 * sin(theta2 + theta4) - 174.15 * sin(theta2 + theta4 + theta5)) * sin(theta1),
         -174.15 * sin(theta1) * sin(theta2 + theta4 + theta5), 0],

        [0, 200.0 * sin(theta2) + 50.0 * cos(theta2) - 200.0 * cos(theta2 + theta4) + 174.15 * cos(theta2 + theta4 + theta5),
         -200.0 * cos(theta2 + theta4) + 174.15 * cos(theta2 + theta4 + theta5),
         174.15 * cos(theta2 + theta4 + theta5), 0],

        [0, 0, 0, 0, 1],

        [0, 1, 1, 1, 0]],
        dtype="float64"
    )


def fk_wx200(thetas):
    dh = [
        [0, 0, 113.25, thetas[0]],
        [pi / 2, 0, 0, thetas[1] - pi / 2],
        [0, 200, 0, pi / 2],
        [0, 50, 0, thetas[2] - pi],
        [0, 200, 0, thetas[3] - pi / 2],
        [pi / 2, 0, 0, thetas[4]],
        [0, 0, 174.15, 0]
    ]

    T = fk_transform(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
    for i in range(1, 7):
        T = T @ fk_transform(dh[i][0], dh[i][1], dh[i][2], dh[i][3])

    return [T[0, 3], T[1, 3], T[2, 3]]


def inverse_kinematics_stuff():
    # Constants and stuff
    thetas = np.array([np.pi, np.pi, np.pi, np.pi, np.pi], dtype="float64")
    x_max = np.array([0.01, 0.01, 0.01, 0.001, 0.001], dtype="float64")
    x_max *= 10

    # Numpy print settings and stuff
    np.set_printoptions(linewidth=np.inf)

    # goal_pose = np.array([212.075, 0.0, 313.25, 0, 0], dtype="float64")
    goal_pose = np.array([0.0, 0.0, 737.4, 0, 0], dtype="float64")
    print("Goal Pose: ", goal_pose)
    t1 = []
    t2 = []
    t4 = []
    t5 = []
    t6 = []
    jdets = []

    x = []
    y = []
    z = []
    roll = []
    pitch = []

    frames = []

    # Matplotlib stuff
    plt.ion()
    fig, ax = plt.subplots()
    line1, = ax.plot(frames, t1, label="Theta 1")
    line2, = ax.plot(frames, t2, label="Theta 2")
    line4, = ax.plot(frames, t4, label="Theta 4")
    line5, = ax.plot(frames, t5, label="Theta 5")
    line6, = ax.plot(frames, t6, label="Theta 6")

    plt.title("Joint angles vs. Time Step", fontsize=25)
    plt.xlabel("Step", fontsize=18)
    plt.ylabel("Joint Angles", fontsize=18)

    # Control loop stuff
    moving = True
    frame = 0
    while moving and frame < 5000:
        # STEP 0
        xyz = fk_wx200(thetas)
        current_pose = np.array([*xyz, 0, 0], dtype="float64")
        print("\nCurrent Pose: ", current_pose)

        # STEP 1
        total_linear_change = goal_pose - current_pose
        print("Total Linear Change: ", total_linear_change)

        if (np.abs(total_linear_change) > 0.1).any():
            # STEP 2
            current_linear_change = (total_linear_change / np.linalg.norm(total_linear_change)) * x_max
            # print("Current Linear Change: ", current_linear_change)

            # STEP 3
            J = jacobian(*thetas)
            J_det = np.linalg.det(J)

            # print("Jacobian: ", J)
            if abs(J_det) > 0.01:
                rotational_change = (np.linalg.inv(J) @ current_linear_change)
                # print("Rotational Change: ", rotational_change)

                # Collect data
                t1.append(thetas[0])
                t2.append(thetas[1])
                t4.append(thetas[2])
                t5.append(thetas[3])
                t6.append(thetas[4])
                jdets.append(abs(J_det))
                x.append(xyz[0])
                y.append(xyz[1])
                z.append(xyz[2])
                roll.append(rotational_change[3])
                pitch.append(rotational_change[4])

                frames.append(frame)
                frame += 1

                line1.set_xdata(frames)
                line1.set_ydata(t1)

                plt.pause(0.000001)

                # plt.clf()
                # plt.plot(frames, t1, label="Theta 1")
                # plt.plot(frames, t2, label="Theta 2")
                # plt.plot(frames, t4, label="Theta 4")
                # plt.plot(frames, t5, label="Theta 5")
                # plt.plot(frames, t6, label="Theta 6")
                # plt.legend()
                # plt.pause(0.000001)

                # Update thetas to new position
                thetas += rotational_change
                # print("New Thetas: ", thetas)

            else:
                moving = False
        else:
            moving = False

    # time_steps = np.linspace(0, 1, len(t1)).tolist()
    #
    # # Animate some stuff I guess
    # fig = plt.figure()
    # plt.plot(time_steps, t1, label="Theta 1")
    # plt.plot(time_steps, t2, label="Theta 2")
    # plt.plot(time_steps, t4, label="Theta 4")
    # plt.plot(time_steps, t5, label="Theta 5")
    # plt.plot(time_steps, t6, label="Theta 6")
    # plt.legend()
    #
    # fig = plt.figure()
    # plt.plot(x, z)
    #
    # fig = plt.figure()
    # plt.plot(time_steps, jdets)
    # plt.ylim([0, max(jdets)])
    # plt.show()


def main():
    # inverse_kinematics_stuff()
    sympy_stuff()


if __name__ == "__main__":
    main()
