#!/usr/bin/env python3
from __future__ import division, print_function

import math
import threading
import boto3
import time
import json

import rospy
import tkinter as tk

import niryo_moveit.msg as niryo_moveit_msgs
from tf.transformations import quaternion_from_euler

"""
pick_pose: 
  position: 
    x: 0.150008633733
    y: -0.199993982911
    z: 0.643720209599
  orientation: 
    x: -1.72476975422e-05
    y: -0.707106769085
    z: 1.72476975422e-05
    w: -0.707106769085
place_pose: 
  position: 
    x: 0.15000000596
    y: 0.20000000298
    z: 0.639999985695
  orientation: 
    x: -0.499999970198
    y: -0.499999970198
    z: 0.499999970198
    w: -0.499999970198
"""

client = boto3.client(
    'sqs',
    aws_access_key_id="AKIAVUTRTKFWBMKMKMNG", 
    aws_secret_access_key="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", 
    region_name="ap-southeast-1"
)
queue_url = "https://sqs.ap-southeast-1.amazonaws.com/387859042668/pose-detection.fifo"

class Controller:
    def __init__(self):
        rospy.init_node("controller_remote", anonymous=True)
        self._end_effector_pose_pub = rospy.Publisher(
            "niryo_end_effector_pose", niryo_moveit_msgs.EndEffectorPose, queue_size=1
        )

        # threading to publish pose
        self.r = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            response = client.receive_message(
                MaxNumberOfMessages=1,
                QueueUrl=queue_url
            )
            if "Messages" in response:
                messages = response["Messages"]
                if len(messages) > 0:
                    message = messages[0]
                    receipt_handle = message["ReceiptHandle"]
                    body = json.loads(message["Body"])
                    client.delete_message(
                        QueueUrl=queue_url,
                        ReceiptHandle=receipt_handle
                    )

                    pos_control_values = body["data"]
                    gripper_state = body["state"]
                    print(pos_control_values, gripper_state)

                    pose = niryo_moveit_msgs.EndEffectorPose()

                    # pos
                    pose.target.position.x = pos_control_values[0]
                    pose.target.position.y = pos_control_values[1]
                    pose.target.position.z = pos_control_values[2]

                    # orientation
                    yaw = pos_control_values[3]
                    pitch = pos_control_values[4]
                    roll = pos_control_values[5]
                    q = quaternion_from_euler(yaw, pitch, roll)
                    pose.target.orientation.x = q[0]
                    pose.target.orientation.y = q[1]
                    pose.target.orientation.z = q[2]
                    pose.target.orientation.w = q[3]

                    # gripper
                    pose.gripper_state = gripper_state
                    self._end_effector_pose_pub.publish(pose)

                    print(pose)

            self.r.sleep()


if __name__ == "__main__":
    controller = Controller()
    controller.run()
