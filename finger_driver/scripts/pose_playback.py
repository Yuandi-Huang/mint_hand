#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

import finger_manipulation.srv
import finger_manipulation.msg
import threading
import argparse
import curses
import time
import ast

class ControlNode(Node):
    def __init__(self):
        super().__init__('finger_driver')
        self.position_publisher = self.create_publisher(
            finger_manipulation.msg.GoalPositionBulk, '/goal_position_bulk', self.qos_profile)
        self.position_message = finger_manipulation.msg.GoalPositionBulk()
    
    def setPosition(self, id, position):
        if isinstance(id, int): id [id]
        if isinstance(position, int): position = [position]
        self.position_message.id = id
        self.position_message.position = position
        return self.position_publisher.publish(self.position_message)
    
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=16,
        reliability=QoSReliabilityPolicy.RELIABLE
    )

def playback(stdscr, controlNode, motors, poses, interval):
    stdscr.addstr("Press enter to begin, and any key to quit\n")
    key = stdscr.getch()
    pose_idx = 0
    stdscr.nodelay(True)
    while True:
        key = stdscr.getch()
        if key != -1: break 
        controlNode.setPosition(motors,poses[pose_idx])
        pose_idx = (pose_idx + 1) % len(poses)
        time.sleep(interval)

def main():
    rclpy.init()
    controlNode = ControlNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controlNode)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    parser = argparse.ArgumentParser(description="Publishes /goal_position_bulk values for predefined poses on loop.")
    parser.add_argument("filename", type=str, help="Path to playback text file")
    parser.add_argument("-i","--interval", type=float, default=2, help="Time delay in seconds before changing pose")
    args = parser.parse_args()

    try:
        with open(args.filename, 'r') as file:
            text = file.read()
    except:
        print("Unable to open file.")
        return
    
    text = text.splitlines()
    motors = ast.literal_eval(text[0])
    poses = []
    del text[:2]

    for pose in text:
        try:
            pose = ast.literal_eval(pose)
            poses.append(pose)
            if len(pose) != len(motors): raise Exception
        except:
            print("Invalid file formatting.")
            return
    
    curses.wrapper(playback, controlNode, motors, poses, args.interval)

if __name__ == '__main__':
    main()