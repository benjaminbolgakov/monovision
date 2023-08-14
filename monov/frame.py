import cv2 as cv
import numpy as np
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform
from skimage.transform import EssentialMatrixTransform
from scipy.spatial import cKDTree

"""
Instead of doing feature extraction for every frame, we gonna fed the
features in the previous frame to track the features in the next
frame. When the number of features is dropped below the threshold we
gonna trigger feature extraction. To track the features, my approach
is using Kanade-Lucas Tracking algorithm, simple KLT algorithm
explained below:

1. Detect feature in the first frame. (which we’ve done in the previous step)
2. For each feature compute motion (translation or affine) between consecutive frames.
3. Link motion vector in successive frames to get track for each point.
4. Introduce new points at every m (10 or 15) frames.
5. Track new and old points using steps 1–3.
"""

max_points = 2000

def compute_keypoints(img):
    """
    - Extract keypoints and compute their descriptors.
    """
    orb = cv.ORB_create(max_points) #Maybe not do this every iteration?

    # Detect keypoints and calculate their descriptors
    kps, des = orb.detectAndCompute(img, None) #None: no mask
    return kps, des
    #return np.array([(kp.pt[0], kp.pt[1]) for kp in kps]), des

def extract_keypoints(img):
    """
    - Extract keypoint corners and computes descriptors seperately.
    """
    #orb = cv.ORB_create() #Maybe not do this every iteration?
    orb = cv.ORB_create(max_points) #Maybe not do this every iteration?

    # Detect solid features
    pts = cv.goodFeaturesToTrack(np.mean(img, axis=2).astype(np.uint8),
        maxCorners=max_points, qualityLevel=0.01, minDistance=8)

    # Extract keypoints from features
    kps = [cv.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in pts]

    # Compute keypoint descriptors
    kps, des = orb.compute(img, kps) # KeyPoint object, descriptor list

    # Return array of 2D keypoints and their descriptors
    return np.array([(kp.pt[0], kp.pt[1]) for kp in kps]), des

def match_frames(f1, f2):
    """
    - Match keypoints between frames
    ================================
    f1: Current frame
    f2: Previous frame
    """
    bf = cv.BFMatcher(cv.NORM_HAMMING)
    #Scan for 'k' best matches between frames
    matches = bf.knnMatch(f1.des, f2.des, k=2)
    ret = []
    idx1, idx2 = [], []
    idx1s, idx2s = set(), set()
    for m,n in matches:
        if m.distance < 0.75*n.distance:
            p1 = f1.kps[m.queryIdx]
            p2 = f2.kps[m.trainIdx]
            if m.distance < 32:
                if m.queryIdx not in idx1s and m.trainIdx not in idx2s:
                    idx1.append(m.queryIdx)
                    idx2.append(m.trainIdx)
                    idx1s.add(m.queryIdx)
                    idx2s.add(m.trainIdx)
                    ret.append((p1, p2))

    # Avoid duplicates
    assert(len(set(idx1)) == len(idx1))
    assert(len(set(idx2)) == len(idx2))
    assert len(ret) >= 8
    ret = np.array(ret)
    idx1 = np.array(idx1)
    idx2 = np.array(idx2)
    if ret.shape[0] >= 8 and len(ret) >= 8:
        model, inliers = ransac((ret[:, 0], ret[:, 1]),
            FundamentalMatrixTransform,
            min_samples=8,
            residual_threshold=1.0,
            max_trials=100)
    else:
        model = np.eye(3)
    print("Matches: %d -> %d -> %d -> %d" % (len(f1.des), len(matches), len(inliers), sum(inliers)))
    return idx1[inliers], idx2[inliers], model

def match_frames2(f1, f2):
    """
    - Match keypoints between frames
    ================================
    f1: Current frame
    f2: Previous frame
    """
    ret = []
    idx1, idx2 = [], []
    idx1s, idx2s = set(), set()
    if len(f1.kps) > 6 and len(f2.kps) > 6:
        bf = cv.BFMatcher(cv.NORM_HAMMING)
        #Scan for 'k' best matches between frames
        matches = bf.knnMatch(f1.des, f2.des, k=2)
        good = []
        try:
            for m, n in matches:
                if m.distance < 0.5 * n.distance:
                    good.append(m)
                    pts1 = f1.kps[m.queryIdx]
                    pts2 = f2.kps[m.trainIdx]
                    if m.queryIdx not in idx1s and m.trainIdx not in idx2s:
                        idx1.append(m.queryIdx)
                        idx2.append(m.trainIdx)
                        idx1s.add(m.queryIdx)
                        idx2s.add(m.trainIdx)
                        ret.append((pts1, pts2))
        except ValueError:
            pass
        # Store matched 2D keypoints
        p1 = np.float32([f1.kps[m.queryIdx] for m in good])
        p2 = np.float32([f2.kps[m.trainIdx] for m in good])
        assert(len(set(idx1)) == len(idx1))
        assert(len(set(idx2)) == len(idx2))
        assert len(ret) >= 8
        ret = np.array(ret)
        idx1 = np.array(idx1)
        idx2 = np.array(idx2)
        # print("Matches: %d -> %d" % (len(f1.des), len(matches)))
        return (p1, p2, idx1, idx2, ret)
    else:
        return (None, None, None, None, None)

# ------------------------| Frame |-------------------------------- #
class Frame(object):
    """
    - Stores extracted information such as Keypoints and
      their descriptors, the pose of the camera at that frame etc.
      ============================================================
      NOTE:
      1. Include camera 'K' in init?
    """
    def __init__(self, img, pmap, pose=np.eye(4), tid=None):
        self.pose = np.array(pose) #Identity mtx if none
        if img is not None:
            self.H, self.W = img.shape[0:2]
            #Collect this frame's keypoints+descriptors
            self.kps, self.des = extract_keypoints(img)
            #Init empty array for map-points
            self.pts = [None]*len(self.kps)

            #NOTE : Do this elsewhere? Or do this at all?
            # self.kpsu = undistort(self.kps)
            # self.kpsuu = unproject(self.kpsu)
            self.fid = tid if tid is not None else pmap.add_frame(self)
            # if tid is not None:
            #     self.fid = tid
            # else:
            #     pmap.add_frame(self)

    # NOTE : Extractor Class?
    def convert_to_keypoint(self, pts, size=1):
        """
        TODO ...
        """
        kps = []
        if pts is not None:
            if pts.ndim > 2:
                kps = [cv.KeyPoint(p[0][0], p[0][1], size=size) for p in pts]
            else:
                kps = [cv.KeyPoint(p[0], p[1], size=size) for p in pts]
        return kps
