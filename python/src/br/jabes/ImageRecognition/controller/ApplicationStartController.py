import python.src.br.jabes.ImageRecognition.strategy.HaarCascadeFromOpenCv2Strategy as haarCascadeStrategy
import python.src.br.jabes.ImageRecognition.strategy.FreenectStrategy as freenectStrategy


def start_image_recognition(config):
    if config is None:
        face_cascade = haarCascadeStrategy.define_cascade_classifier('haarcascade_frontalface_default.xml')
        eye_cascade = haarCascadeStrategy.define_cascade_classifier('haarcascade_eye.xml')
        frame_count = 1
        i = 1
        old_frame = None
        positive_path = "/home/jabes/Documentos/ImageRecognitionKinect/frames/positive"
        negative_path = "/home/jabes/Documentos/ImageRecognitionKinect/frames/negative"
    else:
        # config.properties file should have all values above
        pass

    while 1:
        frame = freenectStrategy.get_video()
        depth = freenectStrategy.get_depth()

        if old_frame is None or old_frame is not frame:
            frame_count += 1
            print(frame_count)
            if frame_count == 1 or frame_count % 10 == 0:
                haarCascadeStrategy.save_frame_as_jpg(positive_path, "frame", i, frame)
                i += 1

        haarCascadeStrategy.detect_symbols_or_faces(frame, face_cascade, eye_cascade)

        haarCascadeStrategy.show_rgb_or_depth_image("RGB Image", frame)
        haarCascadeStrategy.show_rgb_or_depth_image("Depth Image", depth)

        old_frame = frame
        old_depth = depth

        # quit program when 'esc' key is pressed
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    cv2.destroyAllWindows()
