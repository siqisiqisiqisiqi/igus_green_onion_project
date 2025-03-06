#!/usr/bin/env python3

import sys
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)

import rospy
import numpy as np

from wrist_camera.msg import Kpt, Kpts
from igus_fruit_packaging.msg import RobotFeedback
from src.utils import angle_thres, camera_to_gripper, gripper_to_base, kpt_matching, kpt_trans


class IgusController():
    def __init__(self):
        rospy.init_node("test_node")

        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)
        # Init wrist camera position
        rospy.Subscriber("/wrist_kpts", Kpts, self.get_wrist_camera_data)

        # Init the transformation between robot frame and world frame
        self.M = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        self.T = np.array([[50], [12], [80]])

        # system home in millimeter
        self.home = np.array([0, 0, 300])

        self.pick_offset = np.array([0, 0, 30])
        self.grasp_position = None
        self.grasp_yaw = None
        self.actual_pose = None
        self.wrist_camera_kpts = None

        # system frequency
        self.rate = rospy.Rate(10)

    def get_wrist_camera_data(self, data):
        try:
            num = data.num
            kpts_data = np.zeros((num, 3))
            for i, kpt in enumerate(data.kpt):
                kpt = np.array(kpt.data)
                kpt = camera_to_gripper(kpt)
                kpts_data[i] = kpt
            self.wrist_camera_kpts = kpts_data
            # rospy.loginfo(f"wrist camera kpts data is {self.wrist_camera_kpts}")
        except:
            # pass
            rospy.loginfo(f"Problem happens!")

    def get_robot_data(self, data):
        self.actual_pose = [data.x, data.y, data.z, data.yaw]
        # rospy.loginfo(f"robot actual position is {self.actual_pose}")

    def kpt_process(self, p, yaw):
        try:
            kpt_base_list = []
            # TODO: what happens if there is no self.wrist_camera_kpts
            for kpt in self.wrist_camera_kpts:
                rospy.loginfo(f"kpt value is {kpt}")
                kpt_base = gripper_to_base(kpt, self.actual_pose)
                rospy.loginfo(f"kpt base is {kpt_base}.")
                kpt_base_list.append(kpt_base)
            rospy.loginfo(f"kpt_base_list is {kpt_base_list}")
            kpt_optimal = kpt_matching(kpt_base_list, p)
            kpt_optimal = kpt_trans(kpt_optimal, yaw)
            return kpt_optimal
        except:
            return None

    # def pick_and_place(self, p, yaw, index=0):
    #     target_p = [-220, -50 + 50 * index, 110]  # [x, y, z]
    #     target_yaw = 0  # orientation

    #     p[0] = p[0] - 5  # x offset
    #     p_offset = p + self.pick_offset
    #     # p = p + np.array([0, 0, -15])

    #     self.robot_move(p_offset, yaw)
    #     rospy.sleep(2)
    #     p = self.kpt_process(p)
    #     # self.robot_move(p, yaw)
    #     # self.gripper_close()
    #     self.robot_move(self.home, yaw)
    #     rospy.sleep(4)
    #     # self.robot_move(target_p, target_yaw)
    #     # self.gripper_close_release()
    #     # self.robot_move(self.home, target_yaw)
    #     # self.gripper_open()
    #     # self.gripper_open_release()

    def run(self):

        index = 0
        while not rospy.is_shutdown():
            p = np.array([0, 0, 100])
            yaw = 0
            self.kpt_process(p, yaw)
            self.rate.sleep()
            # break


if __name__ == "__main__":
    print("start the igus controller")
    client = IgusController()
    rospy.sleep(1)
    client.run()
