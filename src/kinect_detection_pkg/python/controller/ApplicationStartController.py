#!/usr/bin/env python3

import rospy
import kinect_detection_pkg.python.strategy.FreenectStrategy as FreenectStrategy
import kinect_detection_pkg.python.strategy.OpenCVStrategy as OpenCVStrategy
import kinect_detection_pkg.python.strategy.RosTopicStrategy as RosTopicStrategy
from std_msgs.msg import Int32  # PODERÁ ALTERAR DE ACORDO COM O TIPO DA MENSAGEM


def start_image_recognition(config, root_dir):
    if config is None:
        return None
    else:

        # algorithm conditional variables
        frame_count = 0
        i = 1
        old_frame = None
        time_out = int(config.get("detection.symbol.timeout").data) * 30
        time_out_count = time_out

        # paths for training
        positive_path = root_dir + config.get("cascade.input.positive.path").data
        negative_path = root_dir + config.get("cascade.input.negative.path").data

        # path for cascade xml
        cascade_path = root_dir + config.get("cascade.output.symbol_a_xml.path").data

        # Defining cascade classifier
        symbol_a_cascade = OpenCVStrategy.define_cascade_classifier(cascade_path)

        # ROS variables

        rospy.init_node("kinect_sensor")

        topic_name = config.get("ros.publisher.topic.kinect").data
        publisher = RosTopicStrategy.create_publisher(topic_name, 1)
        publisher_msg = Int32()

    # loop condicional, que obedece ao CTRL+C para interremper execução
    while not rospy.is_shutdown():

        frame = FreenectStrategy.get_video()
        depth = FreenectStrategy.get_depth()

        # Check to avoid processing the same frame multiple times.
        if old_frame is None or old_frame is not frame:

            rects_symbol_a = OpenCVStrategy.detect_cascade_in_frame(frame, symbol_a_cascade)
            print(rects_symbol_a)

            # ROS Publisher goes below
            # If it detects rectangles, and is within the timer constraints

            if len(rects_symbol_a) != 0 and time_out_count >= time_out:
                # Sets the message (for now, its a simple 1)
                publisher_msg.data = 1

                # Publishes
                if publisher_msg is not None:
                    RosTopicStrategy.publish(publisher, publisher_msg)

                # Cleans the MSG object
                publisher_msg.data = 0
                time_out_count = 0

            # time_out_count cap
            if time_out_count < time_out:
                time_out_count += 1

            # OpenCVStrategy.draw_rectangles_in_frame(rects_symbol_a, frame)
            # OpenCVStrategy.show_rgb_or_depth_image("Depth Image", depth)
            # OpenCVStrategy.show_rgb_or_depth_image("RGB Image", frame)

        old_frame = frame
        # old_depth = depth

        # quit program when 'esc' key is pressed
        # k = OpenCVStrategy.wait_key(5) & 0xFF
        # if k == 27:
        #     break

    # OpenCVStrategy.destroy_all_windows()
