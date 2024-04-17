#!/usr/bin/env python3

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
#import roslib; roslib.load_manifest('rviz_python_tutorial')
import os

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon, QPixmap
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import maia_gui_v0
from python_rviz_tools import MyViz
import logging

import rospy
import rospkg
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image



#Logging configuration
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
log = logging.getLogger("maia_hmi")

class ExampleApp(QtWidgets.QMainWindow, maia_gui_v0.Ui_MainWindow):
    def __init__(self, parent=None):
        super(ExampleApp, self).__init__(parent)
        self.setupUi(self)

        rospack = rospkg.RosPack()
        self.hmi_root= os.path.dirname(os.path.realpath(__file__))
        self.resources_path=os.path.join(self.hmi_root, "resources/")

        #load your rviz file to be shown in the HMI
        rviz_config_file=rospack.get_path('maia_gui')+"/rviz_configs/your_rviz_config.rviz"

        #initialize rhe Rviz object
        self.rviz_object = MyViz(rviz_config_file)
        self.rviz_main_frame.addWidget(self.rviz_object)

        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()

        #topics subscribed by the HMI
        rospy.Subscriber('/image_worker', Image, self.ergonomics_image_callback)
        rospy.Subscriber('/current_state', String, self.smb_state_callback)
        rospy.Subscriber('/smb_laser_scanner_status', String, self.smb_laser_scanner_status_callback)
        rospy.Subscriber('/smb_global_safety_status', String, self.smb_global_safety_status_callback)
        rospy.Subscriber('/smb_ergonomics_fps', String, self.smb_ergonomics_fps_callback)
        rospy.Subscriber('/smb_ergonomics_rob_distance', String, self.smb_ergonomics_rob_distance_callback)
        rospy.Subscriber('/smb_ergonomics_rating', String, self.smb_ergonomics_rating_callback)
        rospy.Subscriber('/smb_task_progression', Int32, self.smb_task_progression_callback)

        #topics published by the HMI
        self.smb_cmd_publisher = rospy.Publisher('/smb_stateMachine_command', String, queue_size=1)
        self.smb_grip_cmd_publisher = rospy.Publisher('/commandForGripper', String, queue_size=1)

        ##### callbacks for buttons #####
        self.btn_cycle_start.clicked.connect(self.onBtn_cycle_start)

        self.btn_my_button.clicked.connect(self.mycallback)

        self.btn_cycle_stop.clicked.connect(self.onBtn_cycle_stop)
        self.btn_manual_req.clicked.connect(self.onBtn_manual_req)
        self.btn_auto_req.clicked.connect(self.onBtn_auto_req)
        self.btn_gripper_grasp.clicked.connect(self.onBtn_gripper_grasp)
        self.btn_gripper_release.clicked.connect(self.onBtn_gripper_release)
        self.btn_gripper_send_params.clicked.connect(self.onBtn_gripper_send_params)

        log.info("smb_hmi intialization done")

    ##Button callbacks
    def onBtn_cycle_start(self):
        self.smb_cmd_publisher.publish("start")

    def mycallback(self):
        self.label_current_task.setText("by button pressed!")


    def onBtn_cycle_stop(self):
        self.smb_cmd_publisher.publish("stop")

    def onBtn_manual_req(self):
        self.smb_cmd_publisher.publish("manual")

    def onBtn_auto_req(self):
        self.smb_cmd_publisher.publish("auto")

    def onBtn_gripper_grasp(self):
        gripperId=self.sp_gripper_id.value()
        self.smb_grip_cmd_publisher.publish('{"gripper_id":"'+str(gripperId)+'"","action":"grasp"}')

    def onBtn_gripper_release(self):
        gripperId=self.sp_gripper_id.value()
        self.smb_grip_cmd_publisher.publish('{"gripper_id":"'+str(gripperId)+'"","action":"release"}')

    def onBtn_gripper_send_params(self):
        gripperId=str(self.sp_gripper_id.value())
        deviceMode=str(self.sp_deviceMode.value())
        positionTolerance=str(self.sp_positionTolerance.value())
        gripForce=str(self.sp_gripForce.value())
        driveVelocity=str(self.sp_driveVelocity.value())
        shiftPosition=str(self.sp_shiftPosition.value())
        basePosition=str(self.sp_basePosition.value())
        teachPosition=str(self.sp_teachPosition.value())
        workPosition=str(self.sp_workPosition.value())

        commandString='{"gripper_id":"'+gripperId+'","action":"general","controlWord":"1","deviceMode":"'+deviceMode+'","workPieceNo":"0","reserve":"0","positionTolerance":"'+positionTolerance+'","gripForce":"'+gripForce+'","driveVelocity":"'+driveVelocity+'","basePosition":"'+basePosition+'","shiftPosition":"'+shiftPosition+'","teachPosition":"'+teachPosition+'","workPosition":""'+workPosition+'"}'
        self.smb_grip_cmd_publisher.publish(commandString)

    ####   ROS TOPICS CALLBACKS
    #callback on new image received from the ergonomics-human tracking module
    def ergonomics_image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        #if cols > 60 and rows > 60 :
        #    cv2.circle(cv_image, (50,50), 10, 255)
        #cv2.imshow("Image window", cv_image)
        #cv_image = np.array(cv_image).reshape(60,60).astype(np.int32)
        qimage = QImage(cv_image, cv_image.shape[1], cv_image.shape[0],QImage.Format_RGB888)
        pixmap = QPixmap(qimage)
        pixmap = pixmap.scaled(640,400, Qt.KeepAspectRatio)
        #qimage = QtGui.QImage(cv_image, cv_image.shape[0],cv_image.shape[1],QtGui.QImage.Format_RGB32)

        #img = PrintImage(QPixmap(qimage))
        self.img_ergonomics.setPixmap(QtGui.QPixmap(pixmap))
        #cv2.waitKey(3)

    def smb_ergonomics_rating_callback(self, data):
        if data.data == "good":
            self.renderErgoGood()
        if data.data == "bad":
            self.renderErgoBad()

    def smb_laser_scanner_status_callback(self,data):
        if data.data == "normal":
            self.renderLaserScannerStateNormal()
        if data.data == "triggered":
            self.renderLaserScannerStateTriggered()

    def smb_global_safety_status_callback(self,data):
        if data.data == "ok":
            self.renderSafetyOk()
        if data.data == "warn":
            self.renderSafetyWarn()
        if data.data == "stop":
            self.renderSafetyStop()

    def smb_state_callback(self, data):
        #rospy.logwarn(data.data)
        self.label_current_task.setText(data.data)
        if data.data == "ready":
            self.renderReady()
        if data.data == "error":
            self.renderError()
        if data.data == "running":
            self.renderRunning()
        if data.data == "Unwrinkle":
            self.renderInProgress()
        if data.data == "PnP":
            self.renderInProgress()
        if data.data == "Sewing":
            self.renderInProgress()
        if data.data == "ReconfigureGrasp":
            self.renderInProgress()
        if data.data == "Error":
            self.renderNotReady()

        if data.data == "manual":
            self.renderManual()
        if data.data == "auto":
            self.renderAuto()

    def smb_ergonomics_fps_callback(self,data):
        self.label_ht_fps.setText(data.data)

    def smb_ergonomics_rob_distance_callback(self,data):
        self.label_human_distance.setText(data.data)

    def smb_task_progression_callback(self, data):
        self.task_progressBar.setValue(data.data)

    #HMI FUNCTIONS TO RENDER THE CORRESPONDING
    #ACTION BY HIDING/SHOWING THE PROPER WIDGTES
    def renderHumanTrackingOk(self):
        self.label_saf_human_track_ok.setHidden(False)
        self.label_saf_human_track_warn.setHidden(True)
        self.label_saf_human_track_error.setHidden(True)
        self.label_saf_human_track_unknown.setHidden(True)

    def renderHumanTrackingWarn(self):
        self.label_saf_human_track_ok.setHidden(True)
        self.label_saf_human_track_warn.setHidden(False)
        self.label_saf_human_track_error.setHidden(True)
        self.label_saf_human_track_unknown.setHidden(True)

    def renderHumanTrackingError(self):
        self.label_saf_human_track_ok.setHidden(True)
        self.label_saf_human_track_warn.setHidden(True)
        self.label_saf_human_track_error.setHidden(False)
        self.label_saf_human_track_unknown.setHidden(True)

    def renderErgoGood(self):
        self.label_ergo_ok.setHidden(False)
        self.label_ergo_bad.setHidden(True)
        self.label_ergo_unknown.setHidden(True)

    def renderErgoBad(self):
        self.label_ergo_ok.setHidden(True)
        self.label_ergo_bad.setHidden(False)
        self.label_ergo_unknown.setHidden(True)

    def renderLaserScannerStateNormal(self):
        self.label_saf_scanner_ok.setHidden(False)
        self.label_saf_scanner_stop.setHidden(True)
        self.label_saf_scanner_unknown.setHidden(True)

    def renderLaserScannerStateTriggered(self):
        self.label_saf_scanner_ok.setHidden(True)
        self.label_saf_scanner_stop.setHidden(False)
        self.label_saf_scanner_unknown.setHidden(True)

    def renderSafetyOk(self):
        self.label_saf_glob_ok.setHidden(False)
        self.label_saf_glob_warn.setHidden(True)
        self.label_saf_glob_stop.setHidden(True)
        self.label_saf_global_unkown.setHidden(True)

    def renderSafetyWarn(self):
        self.label_saf_glob_ok.setHidden(True)
        self.label_saf_glob_warn.setHidden(False)
        self.label_saf_glob_stop.setHidden(True)
        self.label_saf_global_unkown.setHidden(True)

    def renderSafetyStop(self):
        self.label_saf_glob_ok.setHidden(True)
        self.label_saf_glob_warn.setHidden(True)
        self.label_saf_glob_stop.setHidden(False)
        self.label_saf_global_unkown.setHidden(True)

    def renderAuto(self):
        self.label_sys_manual_mode.setHidden(True)
        self.label_sys_auto_mode.setHidden(False)

    def renderManual(self):
        self.label_sys_manual_mode.setHidden(False)
        self.label_sys_auto_mode.setHidden(True)

    def renderReady(self):
        self.label_sys_ready.setHidden(False)
        self.label_sys_not_ready.setHidden(True)
        self.label_sys_inprogress.setHidden(True)

    def renderNotReady(self):
        self.label_sys_ready.setHidden(True)
        self.label_sys_not_ready.setHidden(False)
        self.label_sys_inprogress.setHidden(True)

    def renderInProgress(self):
        self.label_sys_ready.setHidden(True)
        self.label_sys_not_ready.setHidden(True)
        self.label_sys_inprogress.setHidden(False)

## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    rospy.init_node('maia_gui', anonymous=True)
    
    app = QApplication( sys.argv )
    form = ExampleApp()
    form.show()
    app.exec_()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down maia_gui")
        app.quit()
        print("maia_gui closed. Bye!")
