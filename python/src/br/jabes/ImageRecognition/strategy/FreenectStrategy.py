import freenect
import HaarCascadeFromOpenCv2Strategy as haarCascade


# function to get RGB image from kinect
def get_video():
    array, _ = freenect.sync_get_video()
    array = haarCascade.get_rbg_array_for_frame(array)
    return array


# function to get depth image from kinect
def get_depth():
    array, _ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array
