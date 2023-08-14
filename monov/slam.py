import os
import time
import sys
import cv2 as cv
import numpy as np

from monov.extractor import Extractor
from monov.odometry import Odometry
from monov.frame import Frame, match_frames, match_frames2
from monov.camera import Camera
from monov.pointmap import PointMap

class SLAM(object):
    """
    TODO
    - Performs feature extraction, computes ...
    ==================================================
    resolution = (W,H)
    camera = 'Camera()' instance
    disp2d = ???
    """
    def __init__(self, camera, disp2d=None):
        self.disp2d = disp2d
        self.pmap = PointMap() #Local map
        self.camera = camera
        self.K = self.camera.K #Intrinsic camera matrix
        self.resolution = self.camera.get_resolution()
        self.W, self.H = self.resolution[0], self.resolution[1]
        self.odo = Odometry(self.camera)

    def process_frame(self, img, pose=None):
        """
        Pose: 4x4
        T: 4x4
        """
        start_time = time.time()
        img = cv.resize(img, self.resolution)
        img = self.odo.undistort(img)
        frame = Frame(img, self.pmap) #include K and pose?
        if frame.fid == 0:
            return
        f1 = self.pmap.frames[-1]
        f2 = self.pmap.frames[-2]
        data = match_frames2(f1, f2) #p1, p2, idx1, idx2, ret
        p1, p2 = data[0], data[1]
        if p1 is not None:
            if len(p1) > 20 and len(p2) > 20:
                #Calculate Transformation matrix from matched keypoints
                T, res = self.odo.compute_transf_mat(p1, p2)
                #Calculate new pose
                self.camera.update_pose(T)
        #Homogenized pose
        h_array = np.array([0,0,0,1])
        h_pose = np.concatenate((self.camera.cur_pose, h_array), axis=0)
        pose_list.append(h_pose)
        #Estimate trajectory
        pose_x, pose_y = self.camera.cur_pose[0, 3], self.camera.cur_pose[2, 3]
        trajectory.append((pose_x, pose_y))
