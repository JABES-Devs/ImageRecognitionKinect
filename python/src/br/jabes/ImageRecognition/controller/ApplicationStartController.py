import python.src.br.jabes.ImageRecognition.strategy.OpenCVStrategy as openCvStrategy
import python.src.br.jabes.ImageRecognition.strategy.FreenectStrategy as freenectStrategy


def start_image_recognition(config, root_dir):
    frame_count = 0
    i = 1
    old_frame = None

    if config is None:
        face_cascade = openCvStrategy.define_cascade_classifier('haarcascade_frontalface_default.xml')
        eye_cascade = openCvStrategy.define_cascade_classifier('haarcascade_eye.xml')
        positive_path = "/cascade/input/positive"
        negative_path = "/cascade/input/negative"
    else:
        # face_cascade = openCvStrategy.define_cascade_classifier('haarcascade_frontalface_default.xml')
        # eye_cascade = openCvStrategy.define_cascade_classifier('haarcascade_eye.xml')
        positive_path = root_dir + config.get("cascade.input.positive.path").data
        negative_path = root_dir + config.get("cascade.input.negative.path").data
        cascade_path = root_dir + config.get("cascade.output.symbol_a_xml.path").data
        symbol_a_cascade = openCvStrategy.define_cascade_classifier(cascade_path)
        pass

    while 1:
        frame = freenectStrategy.get_video()
        depth = freenectStrategy.get_depth()

        # if old_frame is None or old_frame is not frame:
        #     frame_count += 1
        #     print(frame_count)
        #     if frame_count == 1 or frame_count % 10 == 0:
        #         openCvStrategy.save_frame_as_jpg(positive_path, "frame", i, frame)
        #         i += 1

        openCvStrategy.detect_cascade_in_frame(frame, symbol_a_cascade)

        openCvStrategy.show_rgb_or_depth_image("RGB Image", frame)
        openCvStrategy.show_rgb_or_depth_image("Depth Image", depth)

        old_frame = frame
        old_depth = depth

        # quit program when 'esc' key is pressed
        k = openCvStrategy.wait_key(5) & 0xFF
        if k == 27:
            break

    openCvStrategy.destroy_all_windows()
