#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import rospy
import numpy as np
from numpy.linalg import norm
from std_msgs.msg import String

from src.utils import angle_thres
from wrist_camera.msg import Kpt, Kpts
from std_msgs.msg import Float32MultiArray
from src.igus_driver import IgusDriverEncoder
from igus_fruit_packaging.msg import RobotFeedback



class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")

        # Init green onion position subscribers
        rospy.Subscriber("/grasp_pose", Float32MultiArray, self.get_grasp_pose)
        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)
        # Init wrist camera position 
        rospy.Subscriber("/wrist_kpts", Kpts, self.get_wrist_camera_data)
        
        # Init igus message publisher to control the robot
        self.robot_pub = rospy.Publisher("igus_message", String, queue_size=10)

        # Init igus driver
        self.encoder = IgusDriverEncoder()

        # Init the transformation between robot frame and world frame
        self.M = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.T = np.array([[50], [12], [80]])

        # system home in millimeter
        self.home = np.array([0, 0, 300])

        self.pick_offset = np.array([0, 0, 30])
        self.grasp_position = None
        self.grasp_yaw = None
        self.actual_pose = None

        # system frequency
        self.rate = rospy.Rate(10)

    def get_wrist_camera_data(self, data):
        pass

    def get_grasp_pose(self, data):
        """_summary_

        Parameters
        ----------
        data : Float32MultiArray
            [x(mm),y(mm),z(mm),yaw(radian)]
        """
        rospy.loginfo(f"data value is {data.data}")
        try:
            grasp_position = np.array(data.data[:3])
            grasp_yaw = -1 * data.data[-1] * 180 / np.pi
            self.grasp_yaw = angle_thres(grasp_yaw)
            # The length of the gripper is 0.18m
            grasp_position = grasp_position.reshape((3, 1))
            grasp_position = self.M @ grasp_position + self.T
            self.grasp_position = grasp_position.squeeze()
            # rospy.loginfo(f"grasp position is {self.grasp_position}")
        except:
            self.grasp_yaw = None
            self.grasp_position = None
            # rospy.loginfo(f"No suitable green onion to pick!")

    def get_robot_data(self, data):
        self.actual_pose = [data.x, data.y, data.z, data.yaw]

    def robot_move(self, p, yaw: int = 0):
        # desired grasp pose in millimeter
        move_message = self.encoder.cartesian_move(p, yaw)

        # In case losing the message
        for i in range(2):
            self.robot_pub.publish(move_message)
            rospy.sleep(0.05)

        desired_pose = [*p, yaw]
        while norm(np.array(self.actual_pose) - np.array(desired_pose)) > 2:
            rospy.sleep(0.05)

    def gripper_open(self):
        message = self.encoder.gripper(D21=1)
        self.robot_pub.publish(message)
        rospy.sleep(0.5)

    def gripper_open_release(self):
        message = self.encoder.gripper(D21=0)
        self.robot_pub.publish(message)
        rospy.sleep(0.5)

    def gripper_close_release(self):
        message = self.encoder.gripper(D20=0)
        self.robot_pub.publish(message)
        rospy.sleep(1)

    def gripper_close(self):
        message = self.encoder.gripper(D20=1)
        self.robot_pub.publish(message)
        rospy.sleep(1)

    def pick_and_place(self, p, yaw, index=0):
        target_p = [-220, -50 + 50 * index, 110]  # [x, y, z]
        target_yaw = 0  # orientation

        p[0] = p[0] - 5 # x offset
        p_offset = p + self.pick_offset
        p = p + np.array([0, 0, -15])

        self.robot_move(p_offset, yaw)
        rospy.sleep(2)
        # self.robot_move(p, yaw)
        # self.gripper_close()
        self.robot_move(self.home, yaw)
        rospy.sleep(4)
        # self.robot_move(target_p, target_yaw)
        # self.gripper_close_release()
        # self.robot_move(self.home, target_yaw)
        # self.gripper_open()
        # self.gripper_open_release()

    def run(self):
        rospy.loginfo(
            f"Robot will start in 3 seconds, make sure no one is around!")
        rospy.sleep(3)

        self.robot_move(self.home)

        index = 0
        while not rospy.is_shutdown():
            if self.grasp_yaw is not None and self.grasp_position is not None:
                p = self.grasp_position
                yaw = self.grasp_yaw

                self.pick_and_place(p, yaw, index)
                index = index + 1 if index < 3 else 0

                self.grasp_yaw = None
                self.grasp_position = None
                # rospy.sleep(2)
                # break
            self.rate.sleep()
            # break


if __name__ == "__main__":
    print("start the igus controller")
    client = IgusController()
    rospy.sleep(1)
    client.run()
