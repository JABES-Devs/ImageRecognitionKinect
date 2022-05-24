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


# Still set to detect eyes and faces. It will need be changed once we have the training it requires.
# {
def detect_cascade_in_frame(frame, symbol_a_cascade):
    try:
        gray = get_gray_array_for_frame(frame)
        # faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        symbols_a = symbol_a_cascade.detectMultiScale(gray, 1.3, 5)
        for (x, y, w, h) in symbols_a:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
            # roi_gray = gray[y:y + w, x:x + w]
            # roi_color = frame[y:y + h, x:x + w]
            # eyes = eye_cascade.detectMultiScale(roi_gray, 1.3, 5)
            # for (ex, ey, ew, eh) in eyes:
            # cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 5)
    except NameError:
        print("Erro ao tentar detectar faces e olhos!")


# }

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
