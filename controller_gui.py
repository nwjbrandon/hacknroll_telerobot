from __future__ import division, print_function

import math
import threading
import time
import json
import boto3

import tkinter as tk

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

class Application(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.master = master
        self.pos_slider_start_pos = (75, 100)
        self.pos_control_values = [0] * 6

        # cm
        self.min_pos = -200
        self.max_pos = 300

        # radian scale by 100
        self.min_rad = -int(math.pi * 100)
        self.max_rad = int(math.pi * 100)

        # pose
        self.start_pose = [50, 0, 182, 0, 200, 0]
        self.gripper_state = 1

        self.create_widgets()

        # threading to publish pose
        # self.r = rospy.Rate(10)
        self.worker = threading.Thread(target=self.publish)
        self.worker.daemon = True
        self.worker.start()

    def create_widgets(self):
        self.x = tk.Scale(self.master, variable=tk.IntVar(), from_=self.min_pos, to=self.max_pos)
        self.x["command"] = lambda x: self.set_pos_values(0, x)
        self.x.place(x=self.pos_slider_start_pos[0] + 50, y=self.pos_slider_start_pos[1])

        self.y = tk.Scale(self.master, variable=tk.IntVar(), from_=self.min_pos, to=self.max_pos)
        self.y["command"] = lambda x: self.set_pos_values(1, x)
        self.y.place(x=self.pos_slider_start_pos[0] + 100, y=self.pos_slider_start_pos[1])

        self.z = tk.Scale(self.master, variable=tk.IntVar(), from_=self.min_pos, to=self.max_pos)
        self.z["command"] = lambda x: self.set_pos_values(2, x)
        self.z.place(x=self.pos_slider_start_pos[0] + 150, y=self.pos_slider_start_pos[1])

        self.yaw = tk.Scale(self.master, variable=tk.IntVar(), from_=self.min_rad, to=self.max_rad)
        self.yaw["command"] = lambda x: self.set_pos_values(3, x)
        self.yaw.place(x=self.pos_slider_start_pos[0] + 200, y=self.pos_slider_start_pos[1])

        self.pitch = tk.Scale(
            self.master, variable=tk.IntVar(), from_=self.min_rad, to=self.max_rad
        )
        self.pitch["command"] = lambda x: self.set_pos_values(4, x)
        self.pitch.place(x=self.pos_slider_start_pos[0] + 250, y=self.pos_slider_start_pos[1])

        self.row = tk.Scale(self.master, variable=tk.IntVar(), from_=self.min_rad, to=self.max_rad)
        self.row["command"] = lambda x: self.set_pos_values(5, x)
        self.row.place(x=self.pos_slider_start_pos[0] + 300, y=self.pos_slider_start_pos[1])

        self.start_btn = tk.Button(self.master, text="Return to Start")
        self.start_btn["command"] = lambda: self.set_pose(self.start_pose)
        self.start_btn.place(x=100, y=250)

        self.start_btn = tk.Button(self.master, text="Open Gripper")
        self.start_btn["command"] = lambda: self.set_gripper_state(1)
        self.start_btn.place(x=250, y=250)

        self.start_btn = tk.Button(self.master, text="Close Gripper")
        self.start_btn["command"] = lambda: self.set_gripper_state(0)
        self.start_btn.place(x=400, y=250)

    def set_pos_values(self, joint_idx, pos):
        self.pos_control_values[joint_idx] = int(pos)

    def set_pose(self, pose):
        self.x.set(pose[0])
        self.y.set(pose[1])
        self.z.set(pose[2])
        self.yaw.set(pose[3])
        self.pitch.set(pose[4])
        self.row.set(pose[5])
        self.pos_control_values = pose[:]

    def set_gripper_state(self, gripper_state):
        self.gripper_state = gripper_state

    def publish(self):
        self.set_pose(self.start_pose)
        while True:
            x = self.pos_control_values[0]
            y = self.pos_control_values[1]
            z = self.pos_control_values[2]
            yaw = self.pos_control_values[3]
            pitch = self.pos_control_values[4]
            roll = self.pos_control_values[5]

            # gripper
            state = self.gripper_state
            data = {
                'data': [x,y,z,yaw,pitch,roll],
                'state': state
            }

            data = json.dumps(data)
            response = client.send_message(QueueUrl=queue_url, MessageBody=data, MessageGroupId='test_group_1')
            print("message sent...", response)
            time.sleep(1)


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("600x400")
    root.resizable(width=False, height=False)
    app = Application(master=root)
    app.mainloop()
