# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI3.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QIcon, QPixmap

import time
import threading
import rospy
from sensor_msgs.msg import Image
import cob_object_detection_msgs.msg._DetectionArray
from geometry_msgs.msg import Twist
# from p2os_msgs.msg import MotorState
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(2111, 1138)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.arm_camera_window = QtWidgets.QLabel(self.centralwidget)
        self.arm_camera_window.setMinimumSize(QtCore.QSize(640, 480))
        self.arm_camera_window.setObjectName("arm_camera_window")
        self.gridLayout_2.addWidget(self.arm_camera_window, 1, 1, 1, 1)
        self.kinect_text = QtWidgets.QLabel(self.centralwidget)
        self.kinect_text.setObjectName("kinect_text")
        self.gridLayout_2.addWidget(self.kinect_text, 0, 0, 1, 1)
        self.kinect_camera_window = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.kinect_camera_window.sizePolicy().hasHeightForWidth())
        self.kinect_camera_window.setSizePolicy(sizePolicy)
        self.kinect_camera_window.setMinimumSize(QtCore.QSize(640, 480))
        self.kinect_camera_window.setObjectName("kinect_camera_window")
        self.gridLayout_2.addWidget(self.kinect_camera_window, 1, 0, 1, 1)
        self.detection_objects_list = QtWidgets.QListView(self.centralwidget)
        self.detection_objects_list.setObjectName("detection_objects")
        self.gridLayout_2.addWidget(self.detection_objects_list, 2, 0, 1, 1)
        self.map_window = QtWidgets.QLabel(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.map_window.sizePolicy().hasHeightForWidth())
        self.map_window.setSizePolicy(sizePolicy)
        self.map_window.setMinimumSize(QtCore.QSize(640, 480))
        self.map_window.setObjectName("map_window")
        self.gridLayout_2.addWidget(self.map_window, 1, 3, 1, 1)
        self.map_text = QtWidgets.QLabel(self.centralwidget)
        self.map_text.setObjectName("map_text")
        self.gridLayout_2.addWidget(self.map_text, 0, 3, 1, 1)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.CCW = QtWidgets.QPushButton(self.centralwidget)
        self.CCW.setObjectName("CCW")
        self.gridLayout_3.addWidget(self.CCW, 0, 2, 1, 1)
        self.STOP = QtWidgets.QPushButton(self.centralwidget)
        self.STOP.setObjectName("STOP")
        self.gridLayout_3.addWidget(self.STOP, 1, 1, 1, 1)
        self.Forward = QtWidgets.QPushButton(self.centralwidget)
        self.Forward.setObjectName("Forward")
        self.gridLayout_3.addWidget(self.Forward, 0, 1, 1, 1)
        self.Right = QtWidgets.QPushButton(self.centralwidget)
        self.Right.setObjectName("Right")
        self.gridLayout_3.addWidget(self.Right, 1, 2, 1, 1)
        self.Backward = QtWidgets.QPushButton(self.centralwidget)
        self.Backward.setObjectName("Backward")
        self.gridLayout_3.addWidget(self.Backward, 2, 1, 1, 1)
        self.Left = QtWidgets.QPushButton(self.centralwidget)
        self.Left.setObjectName("Left")
        self.gridLayout_3.addWidget(self.Left, 1, 0, 1, 1)
        self.CW = QtWidgets.QPushButton(self.centralwidget)
        self.CW.setObjectName("CW")
        self.gridLayout_3.addWidget(self.CW, 0, 0, 1, 1)
        self.Up = QtWidgets.QPushButton(self.centralwidget)
        self.Up.setObjectName("Up")
        self.gridLayout_3.addWidget(self.Up, 2, 0, 1, 1)
        self.Down = QtWidgets.QPushButton(self.centralwidget)
        self.Down.setObjectName("Down")
        self.gridLayout_3.addWidget(self.Down, 2, 2, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout_3, 2, 3, 1, 1)
        self.arm_text = QtWidgets.QLabel(self.centralwidget)
        self.arm_text.setObjectName("arm_text")
        self.gridLayout_2.addWidget(self.arm_text, 0, 1, 1, 1)
        self.functionality = QtWidgets.QListView(self.centralwidget)
        self.functionality.setObjectName("functionality")
        self.gridLayout_2.addWidget(self.functionality, 2, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 2111, 48))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionOpen = QtWidgets.QAction(MainWindow)
        self.actionOpen.setObjectName("actionOpen")
        self.actionSave = QtWidgets.QAction(MainWindow)
        self.actionSave.setObjectName("actionSave")
        self.menuFile.addAction(self.actionOpen)
        self.menuFile.addAction(self.actionSave)
        self.menubar.addAction(self.menuFile.menuAction())

        print "subscribe"
        rospy.init_node('image_listener')
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        rospy.Subscriber('/detected_object', cob_object_detection_msgs.msg.DetectionArray, self.detection_callback)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        # variables
        self.detection_objects = None

        # # for endeffector to read position, orientation, force and torque
        self._t_image = threading.Thread(target=self.image_worker)
        self._t_image.daemon = True
        self._t_image.start()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.arm_camera_window.setText(_translate("MainWindow", "For Arm Camera"))
        self.kinect_text.setText(_translate("MainWindow", "Kinect Camera"))
        self.kinect_camera_window.setText(_translate("MainWindow", "For Kinect Camera"))
        self.map_window.setText(_translate("MainWindow", "For map"))
        self.map_text.setText(_translate("MainWindow", "Map View Goal Selection"))
        self.CCW.setText(_translate("MainWindow", "CCW"))
        self.CCW.pressed.connect(self.show_detection_object)
        self.STOP.setText(_translate("MainWindow", "STOP"))
        self.Forward.setText(_translate("MainWindow", "Forward"))
        self.Right.setText(_translate("MainWindow", "Right"))
        self.Backward.setText(_translate("MainWindow", "Backward"))
        self.Left.setText(_translate("MainWindow", "Left"))
        self.CW.setText(_translate("MainWindow", "CW"))
        self.Up.setText(_translate("MainWindow", "Up"))
        self.Down.setText(_translate("MainWindow", "Down"))
        self.arm_text.setText(_translate("MainWindow", "Arm Camera Image"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.actionOpen.setText(_translate("MainWindow", "Open"))
        self.actionSave.setText(_translate("MainWindow", "Save"))

    def image_callback(self, msg):
        # print 1
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        image = QtGui.QImage(cv2_img.data, cv2_img.shape[1], cv2_img.shape[0], QtGui.QImage.Format_RGB888).rgbSwapped()
        self.arm_camera_window.setPixmap(QtGui.QPixmap.fromImage(image))
        # pixmap = QPixmap('image.jpeg')

    def detection_callback(self, msg):
        # print 3
        # print msg.detections
        self.detection_objects = msg.detections
        # subscriber.unregister()

    def show_detection_object(self):
        sub_once = None
        # sub_once = rospy.Subscriber('/detected_object,', cob_object_detection_msgs.msg.DetectionArray, self.detection_callback, sub_once)

        self.detection_objects_list.setObjectName("detection_objects")
        objects = QtGui.QStandardItemModel()
        self.detection_objects_list.setModel(objects)
        for item in self.detection_objects:
            object = QtGui.QStandardItem(item.label)
            objects.appendRow(object)

    def image_worker(self):
        print 2
        # read endeffector position and orientation
        # sub_once = rospy.Subscriber('/detected_object,', cob_object_detection_msgs.msg.DetectionArray, self.detection_callback)
        # rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        # rospy.spin()

        # rospy.Subscriber(self.address, geometry_msgs.msg.PoseStamped, self.process)
        # rospy.Subscriber("/j2n6s300_driver/out/tool_wrench", geometry_msgs.msg.WrenchStamped, self.process_wrench)
        # rospy.Subscriber('/fiducial_transforms', fiducial_msgs.msg.FiducialTransformArray, self.handle_goal_base_pose)
    def CCWAction(self):
        print 123


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

