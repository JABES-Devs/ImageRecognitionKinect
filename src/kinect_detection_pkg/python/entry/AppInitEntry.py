#!/usr/bin/env python3

import rospy
from jproperties import Properties
import kinect_detection_pkg.config as config
import kinect_detection_pkg.python.controller.ApplicationStartController as app




if __name__ == "__main__":

    jconfig = Properties()

    with open(config.ROOT_DIR + "/resources/image-recognition_config.properties", "rb") as config_file:
        jconfig.load(config_file)

    app.start_image_recognition(jconfig, config.ROOT_DIR)
