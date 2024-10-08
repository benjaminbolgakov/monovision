import cv2 as cv
# import glob
# import sys
# import os

from monov import calibration
from monov.camera import Camera
from monov.cap import Cap
from monov.aruco import Aruco
from monov.odometry import Odometry
from monov.display import OverlayDisplay
from monov.slam import SLAM

calib_src = "calibration/set_example/results/calibration_example.pkl"

# def camera_feed(calib_src, marker_size, resolution):
def camera_feed(config):
    # Fetch configuration
    resolution = config["camera_resolution"]
    marker_size = config["marker_size"]
    calib_src = config["calibration_file"]
    camera = Camera(resolution, calib_src)

    # Create resources
    cap = Cap(resolution, prop_gui=True)
    aruco = Aruco(camera, marker_size)
    display = OverlayDisplay(resolution)

    # Begin process video
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            if cv.waitKey(25) == ord('q'):
                break
            markers, id_list = aruco.scan(frame, True)
            display.aruco_data(frame, markers, id_list)
            running = cap.display('aruco_test', frame)
            if not running:
                break
    # Release cap and close window
    cap.release()
    cv.destroyAllWindows()
