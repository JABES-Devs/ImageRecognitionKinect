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


def define_cascade_classifier(cascade_source):
    try:
        cascade_classifier = cv2.CascadeClassifier(cascade_source)
        return cascade_classifier
    except NameError:
        return 0


def detect_cascade_in_frame(frame, cascade):
    try:
        gray = get_gray_array_for_frame(frame)
        rects = cascade.detectMultiScale(gray, 1.3, 5)
        return rects
    except NameError:
        print("Erro de deteccao usando o cascade .xml")


def draw_rectangles_in_frame(array, frame):
    try:
        for (x, y, w, h) in array:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
    except NameError:
        print("Atencao ao formato de array de retangulos: cada indice deve possuir os valores x,y,w,h")


def get_center_of_rectangle(rect, camw, camh): # rect is an array[x,y,w,h]
    x = int(rect[0] + rect[2]/2)
    y = int(rect[1] + rect[3]/2)

    # Safety: this will avoid having this center point be outside of the frame
    if x < 0:
        x = 0
    if x > camw:
        x = camw
    if y < 0:
        y = 0
    if y > camh:
        y = camh

    return x, y


def get_depth_of_point(point, depth_frame): # point is a tuple(x,y)
    return depth_frame.item(point[1], point[0])
    # Para este frame depth NUMPY, os parametros de X Y são invertidos.


def get_angles_of_point(w, h, hFov, vFov, point):
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


def save_frame_as_jpg(path, filename, i, frame):
    try:
        cv2.imwrite(os.path.join(path, filename + str(i) + ".jpg"), frame)
    except NameError:
        print("Erro ao tentar salvar frame em jpg")


def show_rgb_or_depth_image(window_name, image_array):
    try:
        cv2.imshow(window_name, image_array)
    except NameError:
        print("Erro ao tentar mostrar o video RGB ou de profundidade")


def wait_key(delay):
    return cv2.waitKey(delay)


def destroy_all_windows():
    cv2.destroyAllWindows()
