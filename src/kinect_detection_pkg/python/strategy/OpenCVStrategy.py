#!/usr/bin/env python3

import cv2
import os


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


def save_frame_as_jpg(path, filename, i, frame):
    try:
        cv2.imwrite(os.path.join(path, filename + str(i) + ".jpg"), frame)
    except NameError:
        print("Erro ao tentar salvar frame em jpg")


def show_rgb_or_depth_image(image_type, image_array):
    try:
        cv2.imshow(image_type, image_array)
    except NameError:
        print("Erro ao tentar mostrar o video RGB ou de profundidade")


def wait_key(delay):
    return cv2.waitKey(delay)


def destroy_all_windows():
    cv2.destroyAllWindows()
