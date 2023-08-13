import cv2 as cv
import glob
import pickle
import numpy as np
from utils import *


calib_log_file = "calib_log.txt"

def calibrate(calib_src, val_src, resolution, square_size, chessboard_size,
              output_src, flags=None, interactive=False):
    """
    - Calibration of camera using chessboard pattern.
    Params
    ------
    calib_src [IN] : str = image source/dir to be used for calibration
    val_src [IN] : str = image source/dir to be used for validating calibration
    chessboard_size [IN] : tuple = dimensions of chessboard (x,y)
    output_src [OUT] : pickle = location to save calibration-params to
    flags [IN] : calibration flags to be used, or "None"
    interactive: perform calibration+validation interactively
    """
    active = True
    objpoints = [] #Stores 3D points
    imgpoints = [] #Stores 2D points
    gray = 0
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    axis_boxes = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
               [0,0,-3],[0,3,-3],[3,3,-3],[3,0,3]])
    prev_img_shape = None
    # ///print_data = ["Flags: ", flags]
    # ///self.util.printer(print_data)
    for fname in calib_src:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners [ret = non-zero if all corners found]
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        print("######################")
        print("Finding corners in image " + str(fname))
        print("######################")
        if ret == True:
            objpoints.append(objp)
            #Refine corners
            corners2 = cv.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            #corners2 = cv.find4QuadCornerSubPix(gray, corners,regopm_size)
            imgpoints.append(corners2)
            img = cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
            if interactive and active:
                cv.namedWindow('img', cv.WINDOW_FREERATIO)
                cv.imshow('img',img)
                cv.waitKey(0)

    cv.destroyAllWindows()
    #Calculate instrinsic parameters
    print("######################")
    print("Calibrating:")
    print("######################")
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,
        gray.shape[::-1], None, None, flags=flags)
    #Refined instrinsic matrix
    mtx_n, roi = cv.getOptimalNewCameraMatrix(mtx, dist, resolution, 1, resolution)
    print_data = ["mtx:", mtx, "dist:", dist, "rvecs:", rvecs, "tvecs:", tvecs, "mtx_n:", mtx_n]
    printer(print_data, calib_log_file)
    #Calculate reprojection error based on calibration
    tot_error, error_list = calc_error(objpoints, imgpoints, mtx, dist, rvecs, tvecs)
    print_data = ["OBJPOINTS_LEN:", len(objpoints), "IMGPOINTS_LEN:", len(imgpoints), "ERROR:", tot_error]
    # /// self.util.printer(print_data)
    #Write calibration results (instrinsic parameters) to file
    write_calibration(output_src, ret, mtx, dist, rvecs, tvecs, mtx_n)
    #Perform validation
    print("######################")
    print("Validating:")
    print("######################")
    #Validate calibration using validation-set
    validate(val_src, resolution, square_size, chessboard_size, mtx, dist, rvecs, tvecs, interactive)


def validate(val_src, resolution, square_size, chessboard_size, mtx, dist, rvecs, tvecs, interactive):
    """
    #### Perform validation of camera calibration ####
    """
    objpoints = [] #Stores 3D points
    imgpts_found = [] #Stores 2D points
    imgpts_project = []
    img_error_list = []
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)
    objp = objp * square_size
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    axisBoxes = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
               [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])
    prev_img_shape = None
    rvv = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    error_list = []
    # Loop each validation image
    for i, fname in enumerate(val_src):
        print("==================================")
        print("Image: ", i)
        print("==================================")
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners [ret = non-zero if all corners found]
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        if ret == True:
            objpoints.append(objp)
            #Refine corners
            corners2 = cv.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            imgpts_found.append(corners2)
            ret, rvec, tvec = cv.solvePnP(objp, corners2, mtx, dist)
            imgpts, jac = cv.projectPoints(objp, rvec, tvec, mtx, dist)
            imgpts_project.append(imgpts)
            #Calculate mean reprojection-error of current image:
            r_error_tot_image = 0
            for point in range(len(imgpts)):
                x_error = abs(imgpts[point][0][0] - corners2[point][0][0])
                y_error = abs(imgpts[point][0][1] - corners2[point][0][1])
                #Calculate eucledian distance (projection error for a point)
                distance = np.linalg.norm(imgpts[point]-corners2[point])
                p_error = np.sqrt((x_error**2) + (y_error**2))
                r_error_tot_image += p_error
                print_data = ["x_error:", x_error, "y_error:", y_error,
                    "p_error:", p_error, "distance:", distance]
                printer(print_data, calib_log_file)

            print("*****************************************")
            r_error_mean_image = (r_error_tot_image/len(imgpts))
            print_data = ["Mean projection-error for image " + str(i), r_error_mean_image]
            printer(print_data, calib_log_file)
            img_error_list.append(r_error_mean_image)
            #Render reprojection-points onto image
            frame_pts = img.copy()
            #self.util.project(frame_pts, imgpts_project[i])
            if interactive:
                cv.namedWindow('img-val', cv.WINDOW_FREERATIO)
                cv.imshow('img-val', img)
                #cv.imshow('img-val', frame_pts)
                cv.waitKey(0)

    r_error_mean_set = 0
    for errs in range(len(img_error_list)):
        r_error_mean_set += img_error_list[errs]
        print("Error image ", errs, ": ", img_error_list[errs])
    r_error_mean_set = (r_error_mean_set/len(img_error_list))
    print_data = ["\n=Finished= \nMean projection-error of set:", r_error_mean_set]
    printer(print_data, calib_log_file)
