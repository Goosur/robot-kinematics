from sympy import *


def fk_transform(alpha, a, d, theta):
    return Matrix([
        [cos(theta), -sin(theta), 0, a],
        [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
        [0, 0, 0, 1]
    ])


def main():
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
    pitch = Matrix([[t2 + t4 + t5]])

    pose = Matrix.vstack(p, pitch)

    J = simplify(pose.jacobian(Matrix([t1, t2, t4, t5])))
    print("\nJacobian Matrix")
    pprint(J)
    print("\nJacobian Determinant")
    print(J.det())


if __name__ == "__main__":
    main()
