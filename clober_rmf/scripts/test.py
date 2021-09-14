#!/usr/bin/env python3

import sys
import argparse

import rclpy
from rclpy.node import Node

from rmf_traffic_msgs.msg import ItineraryClear


def main(argv = sys.argv):
    '''
    Example :
    - participant_name: 0
    - itinerary_version: 0
    - topic_name: rmf_traffic/itinerary_clear
    '''

    default_participant_name = 0
    default_itinerary_version = 0
    default_topic_name = 'rmf_traffic/itinerary_clear'

    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--participant_name', default=default_participant_name)
    parser.add_argument('-v', '--itinerary_version', default=default_itinerary_version)
    parser.add_argument('-t', '--topic_name', default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print('participant_name: {}'.format(args.participant_name))
    print('itinerary_version: {}'.format(args.itinerary_version))
    print('topic_name: {}'.format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node('test_node')
    pub = node.create_publisher(ItineraryClear, args.topic_name, 10)

    msg = ItineraryClear()
    msg.participant = args.participant_name
    msg.itinerary_version = args.itinerary_version

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('all done!')


if __name__ == '__main__':
    main(sys.argv)
