#!/usr/bin/env python3

import sys
import argparse

import rclpy
from rclpy.node import Node

from rmf_traffic_msgs.msg import NegotiationNotice, CloberNegotiationNotice


def main():
    rclpy.init()
    node = rclpy.create_node('send_negotiation_notice_node')
    pub = node.create_publisher(NegotiationNotice, "/rmf_traffic/negotiation_notice", 10)

    msg = NegotiationNotice()
    msg.conflict_version = 1
    msg.participants.append(0)
    # msg.participants.append(1)

    target = CloberNegotiationNotice()
    target.robotid = "clober_0"
    target.start = "n1"
    target.end = "n9"
    target.path.append("n1")
    target.path.append("n2")
    target.path.append("n3")
    target.path.append("n6")
    target.path.append("n9")

    msg.robot_info.append(target)

    enemy = CloberNegotiationNotice()
    enemy.robotid = "clober_1"
    enemy.start = "n3"
    enemy.startidx = 0
    enemy.end = "n7"
    enemy.path.append("n3")
    enemy.path.append("n2")
    enemy.path.append("n1")
    enemy.path.append("n4")
    enemy.path.append("n7")

    msg.robot_info.append(enemy)


    for i in range(3) :
        rclpy.spin_once(node, timeout_sec=5.0)
        pub.publish(msg)
        print('publish')
        # rclpy.spin_once(node, timeout_sec=0.5)

    print('all done!')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
