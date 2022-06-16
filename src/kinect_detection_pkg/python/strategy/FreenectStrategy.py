#!/usr/bin/env python3

import freenect
import kinect_detection_pkg.python.strategy.OpenCVStrategy as OpenCVStrategy
import numpy as np


# function to get RGB image from kinect (OPENCV object!)
def get_video():
    array, _ = freenect.sync_get_video()
    array = OpenCVStrategy.get_rbg_array_for_frame(array)
    return array


# function to get depth image from kinect (NUMPY object!)
def get_depth():
    array, _ = freenect.sync_get_depth(format=freenect.DEPTH_REGISTERED)
    # Returns everything in MILLIMETERS!
    # Almost identical to DEPTH.MM, except this already considers the cameras offset of 2.5cm (aligns the images)

    array = array.astype(np.uint16)
    return array

def stop():
    freenect.sync_stop()
