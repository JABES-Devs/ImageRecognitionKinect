#!/usr/bin/env python3

import rospy
import kinect_detection_pkg.python.strategy.FreenectStrategy as FreenectStrategy
import kinect_detection_pkg.python.strategy.OpenCVStrategy as OpenCVStrategy
import kinect_detection_pkg.python.strategy.RosTopicStrategy as RosTopicStrategy
from kinect_detection_pkg.msg import Kinect_output


def start_image_recognition(config, root_dir):

    # GETTING CONFIG VARIABLES FROM THE CONFIGURATION .PROPERTIES FILE (DEFINED IN ENTRY)

    if config is None:
        return "INVALID CONFIG"
    else:

        # PATHS FOR SAVING TRAINING IMAGES
        positive_path = root_dir + config.get("training.input.positive.path").data
        negative_path = root_dir + config.get("training.input.negative.path").data

        # CAMERA ATTRIBUTES (KINECT)
        cam_rgb_fps = int(config.get("camera.rgb.framerate").data)
        cam_rgb_w = int(config.get("camera.rgb.width").data)
        cam_rgb_h = int(config.get("camera.rgb.height").data)
        cam_rgb_hfov = float(config.get("camera.rgb.horizontal.fov.deg").data)
        cam_rgb_vfov = float(config.get("camera.rgb.vertical.fov.deg").data)
        cam_dep_w = int(config.get("camera.depth.width").data)
        cam_dep_h = int(config.get("camera.depth.height").data)
        cam_dep_hfov = float(config.get("camera.depth.horizontal.fov.deg").data)
        cam_dep_vfov = float(config.get("camera.depth.vertical.fov.deg").data)

        # ROS VARIABLES
        rospy.init_node("kinect_sensor")
        topic_name = config.get("ros.publisher.topic").data
        publisher = RosTopicStrategy.create_publisher(topic_name, 1)
        publisher_msg = Kinect_output()

        # DETECTION VARIABLES
        symbol_a_cascade_path = root_dir + config.get("detection.symbol_a.xml.path").data
        symbol_a_cascade = OpenCVStrategy.define_haar_cascade_classifier(symbol_a_cascade_path)
        symbol_a_timeout = int(config.get("ros.publisher.symbol_a.timeout.seconds").data) * cam_rgb_fps
        frameskip = int(config.get("detection.performance.skip.frames").data)

        # timeout = secs * fps = necessary number of frames to send a new rostopic publish
        # timeout_count = number of frames since the last message published

        # CONTROLLER CONDITIONAL VARIABLES
        old_frame = None
        symbol_a_timeout_count = symbol_a_timeout
        frameskip_count = frameskip

    print("Running: looking for symbols.")

    # CONDITIONAL LOOP THAT OBEYS "CTRL+C" TO STOP EXECUTION
    while not rospy.is_shutdown():

        # Getting the latest frames from Kinect
        frame = FreenectStrategy.get_video() # RGB frame - OPENCV object
        depth = FreenectStrategy.get_depth() # GRAYSCALE frame - NUMPY object - every pixel is in MILIMETERS.

        # Check to avoid processing the same frame multiple times. After this check, it updates the "old_frame"
        # This would happen if the LOOP runs faster than Kinect gathers new images.
        if old_frame is None or old_frame is not frame:

            # PERFORMANCE CHECK
            if frameskip_count < frameskip or symbol_a_timeout_count < symbol_a_timeout:
                # FRAMESKIP = only 1 out of N frames will be processed
                # TIMEOUT = Seconds (frames*framerate) until it can publish another message.
                #           Also, from a performance perspective = until it's "worth" looking for symbols again.
                frameskip_count += 1
                symbol_a_timeout_count += 1
            else:
                frameskip_count = 0

                # HAAR CASCADE CLASSIFIER
                # This will get an array of rectangles, where it found the symbol - rects of the form [x,y,w,h]
                # At this point, this code only supports one classifier, and reports just the first rectangle.
                rects_symbol_a = OpenCVStrategy.detect_haar_cascade_in_frame(frame, symbol_a_cascade)

                # ROS PUBLISHER
                # If it has detected rectangles, and is within the timer constraints (to avoid ROS topic flooding)
                if len(rects_symbol_a) != 0:

                    # Gathers information about the detected rectangles

                    # 1 - Get the center point - Still working with RGB camera parameters
                    point = OpenCVStrategy.get_center_of_rectangle(rects_symbol_a[0], cam_rgb_w, cam_rgb_h)
                    print(point) # point is a tuple(x,y)

                    # 2 - Get the angle values for such point - Angles according to RGB camera specs.
                    hAng, vAng = OpenCVStrategy.get_angles_of_point(point, cam_rgb_w, cam_rgb_h, cam_rgb_hfov, cam_rgb_vfov)

                    # 3 - Convert the point from its original RGB context to DEPTH context (resolution and FOV differences)
                    point = OpenCVStrategy.rgb_to_depth_point(hAng, vAng, cam_dep_w, cam_dep_h, cam_dep_hfov, cam_dep_vfov)

                    # 4 - Get the value in depth camera, using the converted point.
                    millimiters = OpenCVStrategy.get_depth_of_point(point, depth)

                    # Sets the message object
                    publisher_msg.cascade_type = 1
                    publisher_msg.distance_mm = millimiters
                    publisher_msg.horizontal_ang = hAng
                    publisher_msg.vertical_ang = vAng
                    publisher_msg.time = rospy.Time.now()

                    # Publishes the message in the TOPIC
                    if publisher_msg is not None:
                        RosTopicStrategy.publish(publisher, publisher_msg)

                    # Cleans the MSG object, and resets timeout counter.
                    publisher_msg = Kinect_output()
                    symbol_a_timeout_count = 0

            # OpenCVStrategy.draw_rectangles_in_frame(rects_symbol_a, frame)
            # OpenCVStrategy.show_rgb_or_depth_image("Depth Image", depth)
            # OpenCVStrategy.show_rgb_or_depth_image("RGB Image", frame)

        old_frame = frame


    # OpenCVStrategy.destroy_all_windows()
    FreenectStrategy.stop()
