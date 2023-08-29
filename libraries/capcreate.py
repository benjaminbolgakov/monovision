from monov.cap import Cap

class CapTest(object):
    ROBOT_LIBRARY_SCOPE = "Global"
    ROBOT_LIBRARY_DOC_FORMAT = "REST"
    # def __init__(self):
    #     pass

    def create_cap_props(self):
        """
        Test prop-setting. Compare '.get_props()' with 'prop_setup'
        NOTE:
        - Take 'vid_src' as parameter?
        """
        vid_src = "../media/test_vid.avi"
        marker_size = 156 # aruco marker size (mm)
        resolution = (1920, 1080)
        camera = Camera(resolution, calib_src)
        cap = Cap(resolution, vid_src)
        props = cap.get_props()
        return props


prop_setup =
