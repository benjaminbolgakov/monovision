
import glob
import sys
import os
#sys.path.append('src')

# from calibration import calibrate
# from camera import Camera
# from odometry import odometry
# from cap import Cap
# from aruco import Aruco
# from display import OverlayDisplay
# from slam import SLAM

#from monovision import aruco, camera, calibration, cap, odometry, display, slam
from monov import calibration
from monov.camera import Camera
from monov.cap import Cap
from monov.aruco import Aruco
from monov.display import OverlayDisplay
from monov.slam import SLAM

#calib_src = "calibration/set_logitech/results/calibration.pkl"
calib_src = "calibration/set_homecam/results/calibration_homecam_1.pkl"

def calibrate_camera():
    calib_img_src = glob.glob("calibration/set_homecam/src/*")
    valid_img_src = glob.glob("calibration/set_homecam/validation/*")
    output_src = "calibration/set_homecam/results/"
    square_size = 20 # chessboard square size (mm)
    board = (8,5) # chessboard dimensions
    resolution = (1920, 1080)
    calibrate(calib_img_src, valid_img_src, resolution, square_size, board, output_src)


def aruco_vid_testing():
    #vid_src = "media/vids/min_dist_zone.avi"
    vid_src = "../Videos/gray_comp.avi"
    marker_size = 156 # aruco marker size (mm)
    resolution = (1920, 1080)
    #resolution = (640, 480)
    camera = Camera(resolution, calib_src)
    odo = Odometry(resolution)
    cap = Cap(resolution, vid_src)
    aruco = Aruco(camera, marker_size)
    display = OverlayDisplay(resolution)
    #Begin process video
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            markers, id_list = aruco.scan(frame, True)
            display.aruco_data(frame, markers, id_list)
            cap.display('aruco_test', frame)

    cap.release()

def aruco_feed_testing():
    marker_size = 156 # aruco marker size (mm)
    resolution = (1920, 1080)
    camera = Camera(resolution, calib_src)
    #odo = Odometry(resolution)
    cap = Cap(resolution)
    aruco = Aruco(camera, marker_size)
    display = OverlayDisplay(resolution)
    #Begin process video
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            markers, id_list = aruco.scan(frame, True)
            display.aruco_data(frame, markers, id_list)
            cap.display('aruco_test', frame)

    cap.release()

def slam_testing():
    # vid_src = "../Videos/gray_comp.avi" # Laptop source
    vid_src = "../Videos/monovision/lab_gray/gray_comp.avi"
    W, H = 1920, 1080
    resolution = (W, H)
    camera = Camera(resolution, calib_src)
    #odo = Odometry(camera)
    cap = Cap(resolution, vid_src)
    slam = SLAM(camera)
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            #Start processing video
            slam.process_frame(frame)
            cap.display('slam_test', frame)

    cap.release()


if __name__ == "__main__":
    #calibrate_camera()
    aruco_feed_testing() #70cm
    #slam_testing()