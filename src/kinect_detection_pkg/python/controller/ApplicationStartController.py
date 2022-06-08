#!/usr/bin/env python3

import rospy
import kinect_detection_pkg.python.strategy.FreenectStrategy as FreenectStrategy
import kinect_detection_pkg.python.strategy.OpenCVStrategy as OpenCVStrategy
import kinect_detection_pkg.python.strategy.RosTopicStrategy as RosTopicStrategy
from kinect_detection_pkg.msg import Kinect_output # PODERÁ ALTERAR DE ACORDO COM O TIPO DA MENSAGEM


def start_image_recognition(config, root_dir):

    # GETTING CONFIG VARIABLES FROM THE CONFIGURATION .PROPERTIES FILE (DEFINED IN ENTRY)

    if config is None:
        return "INVALID CONFIG"
    else:

        # CONTROLLER CONDITIONAL VARIABLES
        frame_count = 0
        i = 1
        old_frame = None
        time_out = int(config.get("detection.symbol.timeout").data) * 30
        time_out_count = time_out

        # PATHS FOR SAVING TRAINING IMAGES
        positive_path = root_dir + config.get("cascade.input.positive.path").data
        negative_path = root_dir + config.get("cascade.input.negative.path").data

        # GETTING THE CASCADE's XML FILES, THEN DEFINING IT AS A CASCADE CLASSIFIER
        cascade_path = root_dir + config.get("cascade.output.symbol_a_xml.path").data
        symbol_a_cascade = OpenCVStrategy.define_cascade_classifier(cascade_path)

        # CAMERA ATTRIBUTES (KINECT)
        cam_rgb_w = int(config.get("camera.rgb.width").data)
        cam_rgb_h = int(config.get("camera.rgb.height").data)
        cam_rgb_hfov = float(config.get("camera.rgb.horizontal.fov.deg").data)
        cam_rgb_vfov = float(config.get("camera.rgb.vertical.fov.deg").data)
        cam_depth_w = int(config.get("camera.rgb.width").data)
        cam_depth_h = int(config.get("camera.rgb.height").data)
        cam_depth_hfov = float(config.get("camera.rgb.horizontal.fov.deg").data)
        cam_depth_vfov = float(config.get("camera.rgb.vertical.fov.deg").data)

        # ROS VARIABLES
        rospy.init_node("kinect_sensor")
        topic_name = config.get("ros.publisher.topic").data
        publisher = RosTopicStrategy.create_publisher(topic_name, 1)
        publisher_msg = Kinect_output()


    # CONDITIONAL LOOP THAT OBEYS "CTRL+C" TO STOP EXECUTION
    while not rospy.is_shutdown():

        # Getting the latest frames from Kinect
        frame = FreenectStrategy.get_video() # RGB frame - OPENCV object
        depth = FreenectStrategy.get_depth() # GRAYSCALE frame - NUMPY object - every pixel is in MILIMETERS.

        # Check to avoid processing the same frame multiple times. After the loop, it updates the "old_frame"
        # This would happen if the LOOP runs faster than Kinect gathers new images.
        if old_frame is None or old_frame is not frame:

            rects_symbol_a = OpenCVStrategy.detect_cascade_in_frame(frame, symbol_a_cascade)
            # print(rects_symbol_a)
            # This will return multiple retangles (if found)
            # At this point, the code is structured to work with a single cascade classifier,
            # and to report/message only the First rectangle in the array.

            # ROS PUBLISHER
            # If it has detected rectangles, and is within the timer constraints (to avoid ROS topic flooding)
            if len(rects_symbol_a) != 0 and time_out_count >= time_out:

                # Gathers information about the detected rectangles
                point = OpenCVStrategy.get_center_of_rectangle(rects_symbol_a[0], cam_rgb_w, cam_rgb_h) # point is a tuple(x,y)
                milimiters = OpenCVStrategy.get_depth_of_point(point, depth)
                hAng, vAng = OpenCVStrategy.get_angles_of_point(cam_rgb_w, cam_rgb_h, cam_rgb_hfov, cam_rgb_vfov, point)
                time = rospy.Time.now()

                # Sets the message object
                publisher_msg.cascade_type = 1
                publisher_msg.distance_mm = milimiters
                publisher_msg.horizontal_ang = hAng
                publisher_msg.vertical_ang = vAng
                publisher_msg.time = time

                # Publishes the message in the TOPIC
                if publisher_msg is not None:
                    RosTopicStrategy.publish(publisher, publisher_msg)

                # Cleans the MSG object
                publisher_msg = Kinect_output()
                time_out_count = 0

            # time_out_count cap
            if time_out_count < time_out:
                time_out_count += 1

            # OpenCVStrategy.draw_rectangles_in_frame(rects_symbol_a, frame)
            # OpenCVStrategy.show_rgb_or_depth_image("Depth Image", depth)
            # OpenCVStrategy.show_rgb_or_depth_image("RGB Image", frame)

        old_frame = frame


    # OpenCVStrategy.destroy_all_windows()
    FreenectStrategy.stop()
