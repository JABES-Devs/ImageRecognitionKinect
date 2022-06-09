#!/usr/bin/env python3

import cv2
import os
import numpy as np

def get_rbg_array_for_frame(array):
    try:
        image = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        return image
    except NameError:
        return 0


def get_gray_array_for_frame(array):
    try:
        image = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
        return image
    except NameError:
        return 0


def define_haar_cascade_classifier(cascade_source):
    try:
        cascade_classifier = cv2.CascadeClassifier(cascade_source)
        return cascade_classifier
    except NameError:
        return 0


def detect_haar_cascade_in_frame(frame, cascade):
    try:
        # Only works with gray frames.
        # The next two parameters are for:
        #     1st for axis scale reduction (performance)
        #     2nd for minimum of neighbor candidates (certainty reinforcement)
        gray = get_gray_array_for_frame(frame)
        rects = cascade.detectMultiScale(gray, 1.3, 5)

        return rects
    except NameError:
        print("Detection error using the cascade.xml")


def draw_rectangles_in_frame(array, frame):
    try:
        for (x, y, w, h) in array:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
    except NameError:
        print("Attention to the rectangles array form: array[ [x,y,w,h], [x,y,w,h], ... ]")


def get_center_of_rectangle(rect, camw, camh): # rect is an array[x,y,w,h]
    try:
        x = int(rect[0] + rect[2]/2)
        y = int(rect[1] + rect[3]/2)

        # Safety: this will avoid having this center point be outside of the frame (and out of bounds for arrays)
        if x < 0:
            x = 0
        if x >= camw:
            x = camw - 1
        if y < 0:
            y = 0
        if y >= camh:
            y = camh - 1

        return x, y
    except NameError:
        print("Please inform a rectangle of the form [x,y,w,h] and camera specs")

def rgb_to_depth_point(hAng, vAng, dep_w, dep_h, dep_hfov, dep_vfov):
    try:
        # This function matches RGB POINT to DEPTH POINT, considering possible differences in RESOLUTION and FOV
        # Within this function, [point is RGB point] and  [X,Y is DEPTH point].

        # The idea here is: angle from the center -> depth pixels (distance) from center -> pixel in the depth resolution.

        # 1- Get the FOV ratio: the point angle (collected in RGB context) relative to the depth's maximum angle (FOV)
        aux = hAng / dep_hfov

        # 2 - Check if the ratio is beyond 1: that would mean the point is outside of depth camera.
        #     In such cases, it will be treated as if the point is at the edge of the the DEPTHs camera FOV - ratio = 1.
        if aux > 1:
            aux = 1
        if aux < -1:
            aux = -1

        # 3 - Multiply the angle ratio by DEPTH resolution - This is considering both FOV and RESOLUTION differences.
        #     Also, we add half the resolution. The value goes from center displacement in pixels, to actual resolution.
        x = int(aux * dep_w + (dep_w/2))

        # 4 - Repeat 2 -> 3 for the Y coordinate
        aux = vAng / dep_vfov
        if aux > 1:
            aux = 1
        if aux < -1:
            aux = -1
        y = int(((aux * dep_h) * -1) + dep_h/2)

        # 5 - Last correction, to avoid "array out of bounds" error.
        #     If something diverges, with all the int() casting, this guarantees the "array index" compatibility.
        if x == dep_w:
            x = x - 1
        if x < 0:
            x = 0

        if y == dep_h:
            y = y - 1
        if y < 0:
            y = 0

        return x, y

    except NameError:
        print("Please inform RGB point angles, and depth camera information.")


def get_depth_of_point(point, depth_frame):
    try:
        # Make sure the point is converted from RGB to DEPTH contexts (use the function rgb_to_depth_point)
        # This DEPTH frame is a NUMPY array. Always good to remember: to get pixel value, the X and Y parameters are swapped.
        return depth_frame.item(point[1], point[0])

    except NameError:
        print("Couldn't get millimeters of informed point. Make sure point to convert point from RGB to DEPTH, and inform the depth frame")


def get_angles_of_point(point, w, h, hFov, vFov):
    try:
        # Getting the center coordinates of frame.
        wCenter = w/2
        hCenter = h/2
        # Transposing point to center.
        x = point[0] - wCenter
        y = (point[1] - hCenter) * -1
        # Getting the angle (in degrees) according to Field Of View degrees

        # Get the angle, round to 2 decimal places and return.
        hAng = (hFov/2) * (x / wCenter)
        hAng = round(hAng, 2)

        vAng = (vFov/2) * (y / hCenter)
        vAng = round(vAng, 2)

        return hAng, vAng
    except NameError:
        print("Please inform the point and camera information.")


def save_frame_as_jpg(path, filename, i, frame):
    try:
        cv2.imwrite(os.path.join(path, filename + str(i) + ".jpg"), frame)
    except NameError:
        print("Error while trying to save image as jpg")


def show_rgb_or_depth_image(window_name, image_array):
    # This will work with both OPENCV frames or NUMPY ndarrays, such as the depth camera output.
    try:
        cv2.imshow(window_name, image_array)
    except NameError:
        print("Error while trying to display RGB or DEPTH video.")


def destroy_all_windows():
    cv2.destroyAllWindows()
