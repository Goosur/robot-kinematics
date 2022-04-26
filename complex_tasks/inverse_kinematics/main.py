from sympy import *
import numpy as np


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
    thetas = np.array([np.pi, np.pi, np.pi, np.pi, np.pi], dtype="float64")
    x_max = np.array([0.01, 0.01, 0.01, 0.001, 0.001], dtype="float64")
    x_max *= 10

    goal_pose = np.array([212.075, 0.0, 313.25, 0, 0], dtype="float64")
    print("Goal Pose: ", goal_pose)

    moving = True
    while moving:
        # STEP 0
        current_pose = np.array([*fk_wx200(thetas), 0, 0], dtype="float64")
        print("Current Pose: ", current_pose)

        # STEP 1
        total_linear_change = goal_pose - current_pose
        print("Total Linear Change: ", total_linear_change)

        if (np.abs(total_linear_change) > 0.1).any():

            # STEP 2
            current_linear_change = (total_linear_change / np.linalg.norm(total_linear_change)) * x_max
            # print("Current Linear Change: ", current_linear_change)

            # STEP 3
            J = jacobian(*thetas)
            # print("Jacobian: ", J)
            if abs(np.linalg.det(J)) > 0.0001:
                rotational_change = (np.linalg.inv(J) @ current_linear_change)
                # print("Rotational Change: ", rotational_change)
                thetas += rotational_change
                # print("New Thetas: ", thetas)
            else:
                moving = False
        else:
            moving = False


def main():
    inverse_kinematics_stuff()


if __name__ == "__main__":
    main()
