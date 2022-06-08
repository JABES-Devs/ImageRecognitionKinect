#!/usr/bin/env python3

import rospy
from kinect_detection_pkg.msg import Kinect_output  # PODERÁ ALTERAR DE ACORDO COM O TIPO DA MENSAGEM


def create_publisher(topic_name, queue_size):
    return rospy.Publisher(topic_name, Kinect_output, queue_size=queue_size)


def publish(publisher, msg):
    if type(msg) is not Kinect_output:
        return None
    else:
        publisher.publish(msg)
