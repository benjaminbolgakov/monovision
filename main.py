"""
TODO:
- Test configuration input: ensure numerical input for marker size etc ..
"""
import cv2 as cv
import glob
import sys
import os
import json
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
from monov.odometry import Odometry
from monov.display import OverlayDisplay
from monov.slam import SLAM

import programs.calibratecamera
import programs.camerafeed
import programs.capturephoto
import programs.recordvideo
import programs.slamvideo
import programs.videosource

#calib_src = "calibration/set_logitech/results/calibration.pkl"
calib_src = "calibration/set_example/results/calibration_example.pkl"

def configure_calibration():
    print("1. Calibration\n")
    print("- Calibrations are stored in the 'calibration' directory.\n")
    while True:
        calib_input = input("Path to calibration file: calibration/")
        calib_src = "calibration/" + calib_input
        file_verified = os.path.isfile(calib_src)
        if file_verified:
            break
        else:
            print("File not found! Try again.\n")
    return calib_src

def configure_resolution():
    print("2. Camera resolution\n")
    #print("- Acceptable resolutions(WxH): 1920x1080 720x480 todo..\n")
    w = int(input("W: "))
    h = int(input("H: "))
    return (w,h)

def configure_markersize():
    print("3. Marker size\n")
    print("- Aruco marker size refers to the length of the squares consisting in the marker, in millimeters.\n")
    marker_size = int(input("Marker size(mm): "))
    return marker_size

def configure():
    config = {}
    config["calibration_file"] = configure_calibration()
    config["camera_resolution"] = configure_resolution()
    config["marker_size"] = configure_markersize()
    # Write configuration to file
    with open('config.json', 'w') as fp:
        json.dump(config, fp)

def print_current_config():
    with open('config.json') as conf_file:
        config = json.load(conf_file)
    print("\n=Current configuration=")
    print(f"Calibration file: {config["calibration_file"]}")
    print(f"Camera resolution: {config["camera_resolution"][0]}x{config["camera_resolution"][1]}")
    print(f"Marker size: {config["marker_size"]}mm\n")
    return config

if __name__ == "__main__":
    # Check for existing configuration file
    config_exists = os.path.isfile('config.json')
    config = {}
    if config_exists:
        config = print_current_config()
    else:
        print("\nNote: No existing configuration file (config.json) found.")
        print("      Create one with option '7) Configure'\n")

    #### Program start ####
    while True:
        choice = input("1) Calibrate camera\n"
                        "2) Camera feed\n"
                        "3) Pre-recorded video\n"
                        "4) SLAM\n"
                        "5) Record video\n"
                        "6) Capture photo\n"
                        "7) Configure\n"
                        "q) Exit\n\nSelect: ")
        os.system('cls')
        if choice == '1':
            print("==== Camera Calibration ====")
            programs.calibratecamera.calibrate_camera()
        elif choice == '2':
            print("==== Camera Feed ====")
            programs.camerafeed.camera_feed(config)
        elif choice == '3':
            print("==== Pre-recorded Video ====")
            programs.videosource.video_source(config)
            # aruco_vid_testing()
        elif choice == '4':
            print("==== SLAM ====")
            # slam_testing()
            programs.slamvideo.slam_video(config)
        elif choice == '5':
            print("==== Record Video ====")
            programs.recordvideo.record_video()
        elif choice == '6':
            print("==== Capture Photo ====")
            programs.capturephoto.capture_photo()
        elif choice == '7':
            configure()
            print("==== Configuration Wizard ====")
        elif choice == 'q':
            break
        else:
            print("Enter a valid choice")
