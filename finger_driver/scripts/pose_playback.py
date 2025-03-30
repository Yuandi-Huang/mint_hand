#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

import finger_manipulation.srv
import finger_manipulation.msg
import threading
import argparse
import time

class ControlNode(Node):
    def __init__(self):
        super().__init__('finger_driver')
        self.position_publisher = self.create_publisher(finger_manipulation.msg.GoalPosition, '/goal_position', self.qos_profile)
        self.position_message = finger_manipulation.msg.GoalPosition()
    
    def setPosition(self, id, position):
        self.position_message.id = id
        self.position_message.position = position
        return self.position_publisher.publish(self.position_message)
    
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=16,
        reliability=QoSReliabilityPolicy.RELIABLE
    )

def main():
    rclpy.init()
    controlNode = ControlNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controlNode)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    parser = argparse.ArgumentParser(description="Publishes /goal_position values for predefined poses on loop.")
    parser.add_argument("filename", type=str, help="Path to playback text file")
    parser.add_argument("-p","--pose_interval", type=float, default=2, help="Time delay in seconds before changing pose")
    args = parser.parse_args()

    try:
        with open(args.filename, 'r') as file:
            text = file.read()
    except:
        print("Unable to open file.")
        return
    
    text = text.splitlines()
    motors = text[0].split(",")
    poses = []
    del text[:2]

    for pose in text:
        try:
            if "[" not in pose or "]" not in pose: raise Exception
            pose = pose.replace("[","")
            pose = pose.replace("]","")
            pose = pose.split(",")
            poses.append(pose)
            if len(pose) != len(motors): raise Exception
        except:
            print("Invalid file formatting.")
            return
    
    pose_idx = 0
    while True:
        for i in range(0,len(motors)):
            controlNode.setPosition(int(motors[i]),int(poses[pose_idx][i]))
        pose_idx = (pose_idx + 1) % len(poses)
        time.sleep(args.pose_interval)

if __name__ == '__main__':
    main()