#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

import finger_manipulation.srv
import finger_manipulation.msg
import threading
import argparse
import asyncio
import sys
import curses
import time

NUM_MOTORS = 16

class ControlNode(Node):
    def __init__(self):
        super().__init__('finger_driver')
        self.position_client = self.create_client(finger_manipulation.srv.GetPosition, '/get_position')
        self.get_motor_status_client = self.create_client(finger_manipulation.srv.GetTorqueEnabled, '/get_torque_enabled')
        self.position_request = finger_manipulation.srv.GetPosition.Request()        
        self.get_motor_status_request = finger_manipulation.srv.GetTorqueEnabled.Request()
    
    async def getMotorStatus(self, id):
        self.get_motor_status_request.id = id
        future = self.get_motor_status_client.call_async(self.get_motor_status_request)
        await future
        return None if future.result().disconnected else future.result().enabled

    async def getPosition(self, id):
        self.position_request.id = id
        future = self.position_client.call_async(self.position_request)
        await future
        return future.result().position

    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=16,
        reliability=QoSReliabilityPolicy.RELIABLE
    )

def recordDiscrete(controlNode, motors, poses):
    key = sys.stdin.read(1)
    while key != "q":
        pose = []
        for ID in motors:
            pose.append(asyncio.run(controlNode.getPosition(id=ID)))
        poses.append(pose)
        print("Saved:", pose)
        key = sys.stdin.read(1)

def recordContinuous(stdscr, controlNode, motors, poses, interval):
    stdscr.nodelay(True)
    stdscr.scrollok(True)
    while True:
        key = stdscr.getch()
        if key != -1: break
        pose = []
        for ID in motors:
            pose.append(asyncio.run(controlNode.getPosition(id=ID)))
        poses.append(pose)
        stdscr.addstr("Saved: " + str(pose) + "\n")
        time.sleep(interval)

def main():
    rclpy.init()
    controlNode = ControlNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controlNode)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    parser = argparse.ArgumentParser(description="Saves user-configured poses to a text file for playback.")
    parser.add_argument("filename", type=str, help="Path to output file")
    parser.add_argument("-i", "--interval", type=float, default=None, help="Time delay in seconds to automatically record pose")
    args = parser.parse_args()

    motors = []
    poses = []
    for ID in range(1,NUM_MOTORS+1):
        motor_status = asyncio.run(controlNode.getMotorStatus(ID))
        if motor_status is None: continue
        motors.append(ID)

    print("Detected motors:", motors)
    if args.interval is None:
        print("Press enter to save current pose, or q to quit")
        recordDiscrete(controlNode, motors, poses)
    else: 
        print("Press enter to begin recording, and any key to quit")
        input()
        curses.wrapper(recordContinuous, controlNode, motors, poses, args.interval)

    with open(args.filename, "w") as file:
        file.write(str(motors)+"\n---\n")
        for pose in poses:
            file.write(str(pose)+"\n")

if __name__ == '__main__':
    main()