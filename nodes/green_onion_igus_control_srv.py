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
from scipy.spatial.transform import Rotation as R

from src.igus_driver import IgusDriverEncoder
from igus_fruit_packaging.msg import RobotFeedback
from src.utils import angle_thres, camera_to_gripper, gripper_to_base, kpt_matching, kpt_trans
from wrist_camera.srv import ObjectDetection
from tof_detection.srv import EthObjectDetection


class IgusController():
    def __init__(self):
        rospy.init_node("igus_controller")

        # Init robot feedback subscribers
        rospy.Subscriber("/actual_position", RobotFeedback,
                         self.get_robot_data)

        # Init igus message publisher to control the robot
        self.robot_pub = rospy.Publisher("igus_message", String, queue_size=10)

        # Init igus driver
        self.encoder = IgusDriverEncoder()

        # Init the transformation between robot frame and world frame
        euler = [-179.8, 0.4, -1.2]
        r = R.from_euler("xyz", euler, degrees=True)
        self.M = r.as_matrix()
        self.T = np.array([[56], [12], [80]])

        self.home = np.array([0, 0, 300])
        self.pick_offset = np.array([0, 0, 30])

        self.grasp_position = None
        self.grasp_yaw = None
        self.actual_pose = None
        self.wrist_camera_kpts = None

        # service initialization
        rospy.wait_for_service("detect_object")
        self.detect_object = rospy.ServiceProxy(
            "detect_object", ObjectDetection)

        # ETH camera service initialization
        rospy.wait_for_service("eth_detect_object")
        self.eth_detect_object = rospy.ServiceProxy(
            "eth_detect_object", EthObjectDetection)

        # system frequency
        self.rate = rospy.Rate(10)

    def get_robot_data(self, data):
        # [mm, mm, mm, degree]
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

    def conveyor_move(self):
        message = self.encoder.conveyor(D22=1)
        self.robot_pub.publish(message)
        rospy.sleep(3)
        message = self.encoder.conveyor(D22=0)
        rospy.sleep(0.5)

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

    def kpt_process(self, p, yaw):
        response = self.detect_object()
        if len(response.x) == 0:
            rospy.loginfo(0.5)
            response = self.detect_object()
        if len(response.x) == 0:
            rospy.loginfo(f"Wrist camera can't detect the green onion!")
            return None
        x = response.x
        y = response.y
        z = response.z
        wrist_camera_kpts = np.array([x, y, z]).T
        self.wrist_camera_kpts = camera_to_gripper(wrist_camera_kpts)
        kpt_base_list = []
        for kpt in self.wrist_camera_kpts:
            kpt_base = gripper_to_base(kpt, self.actual_pose)
            kpt_base_list.append(kpt_base)
        kpt_optimal = kpt_matching(kpt_base_list, p)
        kpt_optimal = kpt_trans(kpt_optimal, yaw, offset=180)
        return kpt_optimal

    def eth_detection(self):
        res = self.eth_detect_object()
        try:
            if res.z == 0:
                self.grasp_yaw = None
                self.grasp_position = None
            else:
                grasp_position = np.array([res.x, res.y, res.z])
                grasp_yaw = -1 * res.yaw * 180 / np.pi
                self.grasp_yaw = angle_thres(grasp_yaw)
                grasp_position = grasp_position.reshape((3, 1))
                grasp_position = self.M @ grasp_position + self.T
                self.grasp_position = grasp_position.squeeze()
        except:
            self.grasp_yaw = None
            self.grasp_position = None

    def pick_and_place(self, p, yaw, index=0):
        target_p = [-220, -50 + 20 * index, 110]  # [x, y, z]
        target_yaw = 90  # orientation

        p_offset = kpt_trans(p, yaw, offset=-60)
        p_offset = p_offset + self.pick_offset

        self.robot_move(p_offset, yaw)
        rospy.sleep(0.5)
        p = self.kpt_process(p, yaw)
        self.robot_move(p, yaw)
        self.gripper_close()
        self.robot_move(self.home, yaw)
        self.robot_move(target_p, target_yaw)
        self.gripper_close_release()
        self.robot_move(self.home, target_yaw)
        self.gripper_open()
        self.gripper_open_release()

    def run(self):
        rospy.loginfo(
            f"Robot will start in 3 seconds, make sure no one is around!")
        rospy.sleep(3)

        self.robot_move(self.home)

        index = 0
        while not rospy.is_shutdown():
            self.eth_detection()
            if self.grasp_yaw is not None and self.grasp_position is not None:
                p = self.grasp_position
                yaw = self.grasp_yaw

                self.pick_and_place(p, yaw, index)
                index = index + 1
                if index >= 6:
                    rospy.loginfo(f"Completed the pick-and-place process!")
                    self.conveyor_move()
                    break

                self.grasp_yaw = None
                self.grasp_position = None
                rospy.loginfo(f"successfully pick and place one green onion!")

                rospy.sleep(1)
            else:
                rospy.loginfo("Not detect the green ONION!")
            self.rate.sleep()
            # break


if __name__ == "__main__":
    print("start the igus controller")
    node = IgusController()
    rospy.sleep(1)
    node.run()
