# import the necessary modules
import freenect
import cv2
import numpy as np
import os


# def setup_kinect_instance():
# context = freenect.init()
# device = freenect.open_device(context, 0)
# freenect.set_depth_mode(device, freenect.RESOLUTION_MEDIUM, freenect.DEPTH_11BIT)
# freenect.set_video_mode(device, freenect.RESOLUTION_HIGH, freenect.VIDEO_RGB)

# function to get RGB image from kinect
def get_video():
    array, _ = freenect.sync_get_video()
    array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


# function to get depth image from kinect
def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array


# scikit-image

if __name__ == "__main__":

    # setup_kinect_instance()

    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

    frame_count = 1
    i = 98
    old_frame = None
    path = "/cascade/negative"

    while 1:
        # get a frame from RGB camera
        frame = get_video()
        # get a frame from depth sensor
        depth = get_depth()

        if old_frame is None or old_frame is not frame:
            frame_count += 1
            print(frame_count)
            if frame_count == 1 or frame_count % 10 == 0:
                cv2.imwrite(os.path.join(path, 'frame%d.jpg' % i), frame)
                i += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
            roi_gray = gray[y:y + w, x:x + w]
            roi_color = frame[y:y + h, x:x + w]
            eyes = eye_cascade.detectMultiScale(roi_gray, 1.3, 5)
            for (ex, ey, ew, eh) in eyes:
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 5)

        # display RGB image
        cv2.imshow('RGB image', frame)
        # display depth image
        cv2.imshow('Depth image', depth)

        old_frame = frame
        old_depth = depth


        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
