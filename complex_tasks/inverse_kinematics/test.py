from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import sys
import numpy as np
import sympy as sp


def jacobian(theta1, theta2, theta4, theta5, theta6):
    return np.array([
        [
            (-200.0*np.sin(theta2) - 50.0*np.cos(theta2) + 200.0*np.cos(theta2 + theta4) - 174.15*np.cos(theta2 + theta4 + theta5))*np.sin(theta1),
            (-50.0*np.sin(theta2) + 200.0*np.sin(theta2 + theta4) - 174.15*np.sin(theta2 + theta4 + theta5) + 200.0*np.cos(theta2))*np.cos(theta1),
            (200.0*np.sin(theta2 + theta4) - 174.15*np.sin(theta2 + theta4 + theta5))*np.cos(theta1),
            -174.15*np.sin(theta2 + theta4 + theta5)*np.cos(theta1),
            0
        ],
        [
            (200.0*np.sin(theta2) + 50.0*np.cos(theta2) - 200.0*np.cos(theta2 + theta4) + 174.15*np.cos(theta2 + theta4 + theta5))*np.cos(theta1),
            (-50.0*np.sin(theta2) + 200.0*np.sin(theta2 + theta4) - 174.15*np.sin(theta2 + theta4 + theta5) + 200.0*np.cos(theta2))*np.sin(theta1),
            (200.0*np.sin(theta2 + theta4) - 174.15*np.sin(theta2 + theta4 + theta5))*np.sin(theta1),
            -174.15*np.sin(theta1)*np.sin(theta2 + theta4 + theta5),
            0
        ],
        [
            0,
            200.0*np.sin(theta2) + 50.0*np.cos(theta2) - 200.0*np.cos(theta2 + theta4) + 174.15*np.cos(theta2 + theta4 + theta5),
            -200.0*np.cos(theta2 + theta4) + 174.15*np.cos(theta2 + theta4 + theta5),
            174.15*np.cos(theta2 + theta4 + theta5),
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

    T = fk_transform(dh[0][0], dh[0][1], dh[0][2], dh[0][3])
    for i in range(1, 7):
        T = T @ fk_transform(dh[i][0], dh[i][1], dh[i][2], dh[i][3])

    return [T[0, 3], T[1, 3], T[2, 3]]


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)

        self.t1 = []
        self.t2 = []
        self.t4 = []
        self.t5 = []
        self.t6 = []
        self.jdets = []
        self.x = [0]
        self.y = [0]
        self.z = []
        self.roll = []
        self.pitch = []
        self.frames = [0]

        self.x_line = self.graphWidget.plot(self.frames, self.x, pen=pg.mkPen('g'))
        self.y_line = self.graphWidget.plot(self.frames, self.y, pen=pg.mkPen('c'))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        self.frames.append(self.frames[-1] + 1)
        self.x.append(self.frames[-1]**2)
        self.y.append(self.frames[-1])
        self.x_line.setData(self.frames, self.x)
        self.y_line.setData(self.frames, self.y)


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec())
