from monov.cap import Cap

class ArucoTest(object):
    ROBOT_LIBRARY_SCOPE = "Global"
    ROBOT_LIBRARY_DOC_FORMAT = "REST"
    # def __init__(self):
    #     pass


    def calculate_distance_aruco_sample_video(self):
        """
        NOTE:
        - Take 'vid_src' as parameter?
        """
        vid_src = "../media/test_vid.avi"
        marker_size = 156 # aruco marker size (mm)
        resolution = (1920, 1080)
        camera = Camera(resolution, calib_src)
        cap = Cap(resolution, vid_src)
        aruco = Aruco(camera, marker_size)
        distance_results = []
        #Begin process video
        while cap.is_open():
            ret, frame = cap.read()
            if ret:
                markers, id_list = aruco.scan(frame, True)
                distance_results.append(markers[0][0])

        cap.release()
        return distance_results
