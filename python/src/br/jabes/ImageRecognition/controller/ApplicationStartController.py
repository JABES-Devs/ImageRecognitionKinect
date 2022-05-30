import python.src.br.jabes.ImageRecognition.strategy.OpenCVStrategy as openCvStrategy
import python.src.br.jabes.ImageRecognition.strategy.FreenectStrategy as freenectStrategy


def start_image_recognition(config, root_dir):
    if config is None:
        return None
    else:

        # algorithm conditional variables
        frame_count = 0
        i = 1
        old_frame = None

        # paths for training
        positive_path = root_dir + config.get("cascade.input.positive.path").data
        negative_path = root_dir + config.get("cascade.input.negative.path").data

        # path for cascade xml
        cascade_path = root_dir + config.get("cascade.output.symbol_a_xml.path").data

        # Defining cascade classifier
        symbol_a_cascade = openCvStrategy.define_cascade_classifier(cascade_path)

        time_out = int(config.get("detection.symbol.timeout").data) * 30
        time_out_count = time_out

    while 1:

        frame = freenectStrategy.get_video()
        depth = freenectStrategy.get_depth()

        # Check to avoid processing the same frame multiple times.
        if old_frame is None or old_frame is not frame:

            # Procedure to record frames in input folders
            #
            # frame_count += 1
            # print(frame_count)
            # if frame_count == 1 or frame_count % 10 == 0:
            #     openCvStrategy.save_frame_as_jpg(positive_path, "frame", i, frame)
            #     i += 1

            rects_symbol_a = openCvStrategy.detect_cascade_in_frame(frame, symbol_a_cascade)
            # ROS Publisher goes below
            if len(rects_symbol_a) != 0 and time_out_count >= time_out:
                print("Symbol detected")
                time_out_count = 0

            # time_out_count cap
            if time_out_count < time_out:
                time_out_count += 1

            openCvStrategy.draw_rectangles_in_frame(rects_symbol_a, frame)

            openCvStrategy.show_rgb_or_depth_image("Depth Image", depth)
            openCvStrategy.show_rgb_or_depth_image("RGB Image", frame)

        old_frame = frame
        old_depth = depth

        # quit program when 'esc' key is pressed
        k = openCvStrategy.wait_key(5) & 0xFF
        if k == 27:
            break

    openCvStrategy.destroy_all_windows()
