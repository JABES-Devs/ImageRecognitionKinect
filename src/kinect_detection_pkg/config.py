import os as os

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_path_for_file(filename):
    return os.path.join(ROOT_DIR, filename)
