#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32  # PODERÁ ALTERAR DE ACORDO COM O TIPO DA MENSAGEM


def create_publisher(topic_name, queue_size):
    return rospy.Publisher(topic_name, Int32, queue_size=queue_size)


def publish(publisher, msg):
    if type(msg) is not Int32:
        return None
    else:
        publisher.publish(msg)
