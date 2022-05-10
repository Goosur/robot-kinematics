from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import numpy as np
import sympy as sp


# [theta1, theta2, theta4, theta5, theta6]
def jacobian(thetas):
    return np.array([
        [
            (-200.0*np.sin(thetas[1]) - 50.0*np.cos(thetas[1]) + 200.0*np.cos(thetas[1] + thetas[2]) - 174.15*np.cos(thetas[1] + thetas[2] + thetas[3]))*np.sin(thetas[0]),
            (-50.0*np.sin(thetas[1]) + 200.0*np.sin(thetas[1] + thetas[2]) - 174.15*np.sin(thetas[1] + thetas[2] + thetas[3]) + 200.0*np.cos(thetas[1]))*np.cos(thetas[0]),
            (200.0*np.sin(thetas[1] + thetas[2]) - 174.15*np.sin(thetas[1] + thetas[2] + thetas[3]))*np.cos(thetas[0]),
            -174.15*np.sin(thetas[1] + thetas[2] + thetas[3])*np.cos(thetas[0]),
            0
        ],
        [
            (200.0*np.sin(thetas[1]) + 50.0*np.cos(thetas[1]) - 200.0*np.cos(thetas[1] + thetas[2]) + 174.15*np.cos(thetas[1] + thetas[2] + thetas[3]))*np.cos(thetas[0]),
            (-50.0*np.sin(thetas[1]) + 200.0*np.sin(thetas[1] + thetas[2]) - 174.15*np.sin(thetas[1] + thetas[2] + thetas[3]) + 200.0*np.cos(thetas[1]))*np.sin(thetas[0]),
            (200.0*np.sin(thetas[1] + thetas[2]) - 174.15*np.sin(thetas[1] + thetas[2] + thetas[3]))*np.sin(thetas[0]),
            -174.15*np.sin(thetas[0])*np.sin(thetas[1] + thetas[2] + thetas[3]),
            0
        ],
        [
            0,
            200.0*np.sin(thetas[1]) + 50.0*np.cos(thetas[1]) - 200.0*np.cos(thetas[1] + thetas[2]) + 174.15*np.cos(thetas[1] + thetas[2] + thetas[3]),
            -200.0*np.cos(thetas[1] + thetas[2]) + 174.15*np.cos(thetas[1] + thetas[2] + thetas[3]),
            174.15*np.cos(thetas[1] + thetas[2] + thetas[3]),
            0
        ],
        [0, 0, 0, 0, 1],
        [0, 1, 1, 1, 0]
    ], dtype="float64")


def fk_transform(alpha, a, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, a],
        [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha)*d],
        [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha), sp.cos(alpha)*d],
        [0, 0, 0, 1]
    ])


def fk_wx200(thetas):
    dh = [
        [0, 0, 113.25, thetas[0]],
        [np.pi / 2, 0, 0, thetas[1] - np.pi / 2],
        [0, 200, 0, np.pi / 2],
        [0, 50, 0, thetas[2] - np.pi],
        [0, 200, 0, thetas[3] - np.pi / 2],
        [np.pi / 2, 0, 0, thetas[4]],
        [0, 0, 174.15, 0]
    ]

    t = fk_transform(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
    for i in range(1, 7):
        t = t @ fk_transform(dh[i][0], dh[i][1], dh[i][2], dh[i][3])

    return [t[0, 3], t[1, 3], t[2, 3]]


def inverse_kinematics(current_thetas, goal_pose):
    new_thetas = current_thetas[:]

    x_max = np.array([0.01, 0.01, 0.01, 0.01, 0.01], dtype="float64")
    # x_max *= 50

    gripper_position = fk_wx200(current_thetas)
    current_pose = np.array([*gripper_position, current_thetas[4] - np.pi, current_thetas[1] + current_thetas[2] + current_thetas[3] - 3*np.pi], dtype="float64")

    # STEP 1
    total_linear_change = goal_pose - current_pose

    # STEP 2
    current_linear_change = (total_linear_change / np.linalg.norm(total_linear_change)) * x_max

    # STEP 3
    j = jacobian(current_thetas)
    jdet = np.linalg.det(j)

    # CHANGED THIS TO 15000 BECAUSE DETERMINANT IS BIG WHEN IT SPAZZES
    if abs(jdet) > 1:
        rotational_change = (np.linalg.inv(j) @ current_linear_change)
        # Update thetas to new position
        new_thetas = current_thetas + rotational_change

    return new_thetas, jdet


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.canvas = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.canvas)

        # Create subplot for joint angles
        self.joint_plot = self.canvas.addPlot(row=0, col=0)
        # Label joint angles plot
        self.joint_plot.addLegend()
        self.joint_plot.getAxis("left").setLabel("Joint Angle (radians)")
        self.joint_plot.getAxis("bottom").setStyle(showValues=False)

        # Create subplot for jacobian determinant
        self.det_plot = self.canvas.addPlot(row=1, col=0)
        self.det_plot.addLegend()
        self.det_plot.getAxis("bottom").setLabel("Time (steps)")
        self.det_plot.setXLink(self.joint_plot)

        # Create subplot for end effector orientation
        self.orientation_plot = self.canvas.addPlot(row=0, col=1)
        self.orientation_plot.addLegend()
        self.orientation_plot.getAxis("bottom").setStyle(showValues=False)
        self.orientation_plot.getAxis("left").setLabel("End Effector Orientation (radians)")
        self.orientation_plot.setXLink(self.joint_plot)

        # Create subplot for end effector position
        self.position_plot = self.canvas.addPlot(row=1, col=1)
        self.position_plot.addLegend()
        self.position_plot.getAxis("bottom").setLabel("Time (steps)")
        self.position_plot.getAxis("left").setLabel("Position (mm)")
        self.position_plot.setXLink(self.joint_plot)

        # Goal positions
        # self.goal_pose = np.array([156.625, 0.0, 313.15, 0, 0], dtype="float64")
        # self.goal_pose = np.array([0.0, 0.0, 737.4, 0, 0], dtype="float64")
        self.goal_pose = np.array([0.0, 0.0, 600.0, np.pi/2, -np.pi/2], dtype="float64")

        self.t1, self.t2, self.t4, self.t5, self.t6 = [np.pi], [np.pi], [np.pi], [np.pi], [np.pi]
        self.jdets = [abs(np.linalg.det(jacobian(self.t1 + self.t2 + self.t4 + self.t5 + self.t6)))]
        xyz_init = fk_wx200(self.t1 + self.t2 + self.t4 + self.t5 + self.t6)
        self.x_pos, self.y_pos, self.z_pos = [float(xyz_init[0])], [float(xyz_init[1])], [float(xyz_init[2])]
        self.roll = [self.t6[-1] - np.pi]
        self.pitch = [self.t2[-1] + self.t4[-1] + self.t5[-1] - 3*np.pi]
        self.frames = [0]
        # Joint angles graph
        self.t1_line = self.joint_plot.plot(self.frames, self.t1, pen='b', name="Waist")
        self.t2_line = self.joint_plot.plot(self.frames, self.t2, pen='g', name="Shoulder")
        self.t4_line = self.joint_plot.plot(self.frames, self.t4, pen='r', name="Elbow")
        self.t5_line = self.joint_plot.plot(self.frames, self.t5, pen='c', name="Wrist Pitch")
        self.t6_line = self.joint_plot.plot(self.frames, self.t6, pen='m', name="Wrist Roll")
        # Jacobian determinant graph
        self.det_line = self.det_plot.plot(self.frames, self.jdets, 'y', name="Determinant")
        # End effector position graph
        self.x_line = self.position_plot.plot(self.frames, self.x_pos, pen='r', name="X")
        self.y_line = self.position_plot.plot(self.frames, self.y_pos, pen='g', name="Y")
        self.z_line = self.position_plot.plot(self.frames, self.z_pos, pen='b', name="Z")
        # End effector orientation graph
        self.roll_line = self.orientation_plot.plot(self.frames, self.roll, pen='c', name="Roll")
        self.pitch_line = self.orientation_plot.plot(self.frames, self.pitch, pen='m', name="Pitch")

        # Create and begin the timer for redrawing the pyqtgraph plots which simulates animation.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        # Increment the number of simulated time steps
        self.frames.append(self.frames[-1] + 1)
        # Compute the next set of joint angles with inverse kinematics
        new_thetas, new_jdet = inverse_kinematics(
            # Current thetas
            [self.t1[-1], self.t2[-1], self.t4[-1], self.t5[-1], self.t6[-1]],
            # Goal x, y, z, roll, pitch
            self.goal_pose
        )
        # Add the new thetas to the list of simulated thetas for plotting
        self.t1.append(new_thetas[0])
        self.t2.append(new_thetas[1])
        self.t4.append(new_thetas[2])
        self.t5.append(new_thetas[3])
        self.t6.append(new_thetas[4])
        # Add the new determinant to the list of simulated determinants for plotting
        self.jdets.append(abs(new_jdet))
        # Get new end effector position from new thetas
        new_xyz = fk_wx200([self.t1[-1], self.t2[-1], self.t4[-1], self.t5[-1], self.t6[-1]])
        # Add the new xyz positions to the list of simulated xyz positions for plotting
        self.x_pos.append(float(new_xyz[0]))
        self.y_pos.append(float(new_xyz[1]))
        self.z_pos.append(float(new_xyz[2]))
        # Compute the new roll and pitch from the new thetas and add them to the lists of simulated rolls and pitches
        # for plotting
        self.roll.append(new_thetas[4] - np.pi)
        self.pitch.append(new_thetas[1] + new_thetas[2] + new_thetas[3] - 3*np.pi)

        # Update the joint angles plot
        self.t1_line.setData(self.frames, self.t1)
        self.t2_line.setData(self.frames, self.t2)
        self.t4_line.setData(self.frames, self.t4)
        self.t5_line.setData(self.frames, self.t5)
        # Update the determinant plot
        self.det_line.setData(self.frames, self.jdets)
        # Update the position plot
        self.x_line.setData(self.frames, self.x_pos)
        self.y_line.setData(self.frames, self.y_pos)
        self.z_line.setData(self.frames, self.z_pos)
        # Update the orientation plot
        self.roll_line.setData(self.frames, self.roll)
        self.pitch_line.setData(self.frames, self.pitch)


def main(argv):
    app = QtWidgets.QApplication(argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main(sys.argv)
