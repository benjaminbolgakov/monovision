import cv2 as cv
import numpy as np

"""
TODO:
- Compare using 'goodFeaturesToTrack' and 'detectAndCompute' methods.
"""

class Extractor:
    def __init__(self, max_points=2000):
        """
        - Extracts keypoints and computes their descriptors.
        ====================================================
        max_points: Maximum amount of points being returned.
        """
        self.max_points = max_points
        self.orb = cv.ORB_create()
        self.matcher = cv.BFMatcher(cv.NORM_HAMMING)


    def extract_good_keypoints(self, img):
        """
        - Extract good corner-points and compute their descriptors using ORB.
        """
        # Detection
        features = cv.goodFeaturesToTrack(np.mean(img, axis=2).astype(np.uint8),
            maxCorners=max_points, qualityLevel=0.01, minDistance=8)
        # Extract keypoints and compute their descriptors
        kps = [cv.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in features]
        kps, des = self.orb.compute(img, kps)
        return np.array([(kp.pt[0], kp.pt[1]) for kp in kps]), des

    def extract_keypoints(self, img):
        """
        - Extract keypoints and compute their descriptors.
        """
        kps, des = self.orb.detectAndCompute(img, None) #No mask applied
        #Return keypoints as np-array together with descriptors
        return np.array([(kp.pt[0], kp.pt[1]) for kp in kps]), des
