import os
import numpy as np
import cv2 as cv
import sys
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import glob
import pickle
import datetime

separator = "================================================="
log_dir = "logs/"

def printer(data_package, f_name):
    write_package = []
    print(separator)
    write_package.append(separator)
    for data in data_package:
        print(data)
        write_printer(data, f_name)
    write_package.append(separator)
    print(separator)

def write_printer(data, f_name):
    with open(log_dir+f_name, "a") as file:
        file.write(str(data) + "\n")

def fourcc_to_codec(fourcc):
    """
    - Converts 32-bit codec identifier to codec name.
    """
    fourcc_str = "".join([chr((int(fourcc) >> 8 * i) & 0xFF) for i in range(4)])
    return fourcc_str

def get_backend_name(identifier):
    for name, value in cv.__dict__.items():
        if value == identifier:
            return name
    return None

def calc_error(objpoints, imgpoints, mtx, dist, rvecs, tvecs):
    """
    ### Calculate mean reprojection error ####
    """
    mean_error = 0
    error_list = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
        error_list.append(error)
        mean_error += error
    mean_error_tot = (mean_error/len(objpoints))
    return (mean_error_tot, error_list)


def write_calibration(output_src, ret, mtx, dist, rvecs, tvecs, mtx_n):
    f_name = 'calibration.pkl'
    f_full = file_checker(output_src, f_name)
    with open(f_full, 'wb') as f:
        pickle.dump((ret, mtx, dist, rvecs, tvecs, mtx_n), f)

def file_checker(fdir, fname, fext):
    fpath = fdir + fname + fext
    ct = 1
    write = False
    while not write:
        if os.path.exists(fpath):
            fpath = fdir + fname + str(ct) + fext
            ct += 1
        else:
            write = True
            print("Writing to file as: " + str(fpath) + "\n")
    return fpath
