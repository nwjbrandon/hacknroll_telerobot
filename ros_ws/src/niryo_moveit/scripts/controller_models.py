#!/usr/bin/env python3
from __future__ import division, print_function

import math
import threading
import time

import cv2
import numpy as np
import mediapipe as mp
import transforms3d
import rospy

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
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands

def get_left_right_wrist_pos(pose_results):
    # print("num of points:", len(pose_results.pose_world_landmarks.landmark))
    left_wrist_landmark = pose_results.pose_world_landmarks.landmark[15]
    right_wrist_landmark = pose_results.pose_world_landmarks.landmark[16]
    left_wrist_point = [left_wrist_landmark.x, left_wrist_landmark.y, left_wrist_landmark.z, left_wrist_landmark.visibility]
    right_wrist_point = [right_wrist_landmark.x, right_wrist_landmark.y, right_wrist_landmark.z, right_wrist_landmark.visibility]
    left_wrist_point = np.array(left_wrist_point)
    right_wrist_point = np.array(right_wrist_point)
    # print("left wrist:", left_wrist_point)
    # print("right wrist:", right_wrist_point)
    return left_wrist_point, right_wrist_point

def get_thumb_index_wrist_pos(hands_results):
    # print("num of points:", len(hands_results.multi_hand_world_landmarks[0].landmark))
    thumb_tip_landmark = hands_results.multi_hand_world_landmarks[0].landmark[4]
    index_finger_tip_landmark = hands_results.multi_hand_world_landmarks[0].landmark[8]
    wrist_landmark = hands_results.multi_hand_world_landmarks[0].landmark[0]

    thumb_tip_point = [thumb_tip_landmark.x, thumb_tip_landmark.y, thumb_tip_landmark.z, thumb_tip_landmark.visibility]
    index_finger_tip_point = [index_finger_tip_landmark.x, index_finger_tip_landmark.y, index_finger_tip_landmark.z, index_finger_tip_landmark.visibility]
    wrist_point = [wrist_landmark.x, wrist_landmark.y, wrist_landmark.z, wrist_landmark.visibility]


    thumb_tip_point = np.array(thumb_tip_point)
    index_finger_tip_point = np.array(index_finger_tip_point)
    wrist_point = np.array(wrist_point)
    return wrist_point, thumb_tip_point, index_finger_tip_point

def compute_gap_between_thumb_index_fingers_tip(thumb_tip_point, index_finger_tip_point):
    gap_distance = np.linalg.norm(thumb_tip_point - index_finger_tip_point)
    print("gap distance:", gap_distance)
    return gap_distance

# https://itectec.com/matlab/matlab-how-to-calculate-roll-pitch-and-yaw-from-xyz-coordinates-of-3-planar-points/
def compute_hand_orientation(p1, p2, p3):
    p1, p2, p3 = p1[:3], p2[:3], p3[:3] 
    x = (p1 + p2)/2 - p3
    v1, v2 = p2 - p1, p3 - p1
    z = np.cross(v1, v2)
    z = z / np.linalg.norm(z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.concatenate([[x], [y], [z]]).T
    yaw, pitch, roll = transforms3d.euler.mat2euler(R, 'sxyz')
    print("yaw:", yaw)
    print("pitch:", pitch)
    print("roll:", roll)
    return yaw, pitch, roll


class Controller:
    def __init__(self):
        rospy.init_node("controller_models", anonymous=True)
        self._end_effector_pose_pub = rospy.Publisher(
            "niryo_end_effector_pose", niryo_moveit_msgs.EndEffectorPose, queue_size=1
        )

        # hand position
        self.end_effector_position = np.array([0.2, 0.0, 0.91]) # height, depth, width
        self.current_end_effector_position = np.array([0.2, 0.0, 0.91])
        self.curr_hand_position = None

        # hand orientation
        self.end_effector_orientation = np.array([0.0, 1.0, 0.0])
        self.current_end_effector_orientation = np.array([0.0, 1.0, 0.0])
        self.curr_hand_orientation = None
        
        self.gripper_state = 1

        self.r = rospy.Rate(30)

    def run(self):
        time.sleep(3)
        # For webcam input:
        cap = cv2.VideoCapture(0)
        with mp_pose.Pose(
            min_detection_confidence=0.5, 
            min_tracking_confidence=0.5
        ) as pose, mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        ) as hands:
            while cap.isOpened() and not rospy.is_shutdown():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                pose_results = pose.process(image)
                hands_results = hands.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                # Draw the body pose annotation on the image.
                if pose_results.pose_world_landmarks:
                    # TODO: send left and right wrist points
                    left_wrist_point, right_wrist_point = get_left_right_wrist_pos(pose_results)
                    mp_drawing.draw_landmarks(
                        image,
                        pose_results.pose_landmarks,
                        mp_pose.POSE_CONNECTIONS,
                        landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
                    
                    if self.curr_hand_position is None:
                        right_wrist_point = right_wrist_point[:3][[2,0,1]]
                        self.curr_hand_position = right_wrist_point
                    else:
                        right_wrist_point = right_wrist_point[:3][[2,0,1]]
                        diff_hand_position =  (right_wrist_point - self.curr_hand_position) * 1.5
                        diff_hand_position[2] = -diff_hand_position[2]
                        self.current_end_effector_position = self.end_effector_position + diff_hand_position

                # Draw the hand pose annotation on the image.
                if hands_results.multi_hand_landmarks:
                    # TODO: send gap distance and orientation
                    wrist_point, thumb_tip_point, index_finger_tip_point = get_thumb_index_wrist_pos(hands_results)
                    gap_distance = compute_gap_between_thumb_index_fingers_tip(thumb_tip_point, index_finger_tip_point)
                    yaw, pitch, roll = compute_hand_orientation(wrist_point, thumb_tip_point, index_finger_tip_point)

                    for hand_landmarks in hands_results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style())

                    if self.curr_hand_orientation is None:
                        self.curr_hand_orientation = np.array([yaw, pitch, roll])
                    else:
                        wrist_orientation = np.array([yaw, pitch, roll])
                        diff_hand_orientation =  (wrist_orientation - self.curr_hand_orientation)
                        self.current_end_effector_orientation = self.end_effector_orientation + diff_hand_orientation
                    
                    gap_distance /= 0.1
                    if gap_distance < 0:
                        self.gripper_state = 0
                    elif gap_distance > 1:
                        self.gripper_state = 1
                    else:
                        self.gripper_state = gap_distance

                # Flip the image horizontally for a selfie-view display.
                cv2.imshow('MediaPipe Pose', cv2.flip(image, 1))
                if cv2.waitKey(1) & 0xFF == 27:
                    break

                # Publish msg
                msg = niryo_moveit_msgs.EndEffectorPose()

                msg.target.position.x = self.current_end_effector_position[0]
                msg.target.position.y = self.current_end_effector_position[1]
                msg.target.position.z = self.current_end_effector_position[2]

                yaw = self.current_end_effector_orientation[0]
                pitch = self.current_end_effector_orientation[1]
                roll = self.current_end_effector_orientation[2]
                q = quaternion_from_euler(yaw, pitch, roll)
                msg.target.orientation.x = q[0]
                msg.target.orientation.y = q[1]
                msg.target.orientation.z = q[2]
                msg.target.orientation.w = q[3]

                msg.gripper_state = self.gripper_state
                self._end_effector_pose_pub.publish(msg)

                self.r.sleep()
        cap.release()

if __name__ == "__main__":
    controller = Controller()
    controller.run()


