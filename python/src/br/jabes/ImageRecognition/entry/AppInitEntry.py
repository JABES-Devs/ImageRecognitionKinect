import python.src.br.jabes.ImageRecognition.controller.ApplicationStartController as app
from jproperties import Properties
import config

if __name__ == "__main__":

    jconfig = Properties()

    with open(config.ROOT_DIR + "/resources/image-recognition_config.properties", "rb") as config_file:
        jconfig.load(config_file)

    app.start_image_recognition(jconfig, config.ROOT_DIR)
