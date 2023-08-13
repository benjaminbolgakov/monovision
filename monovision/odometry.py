import cv2 as cv
import pickle
import numpy as np

class Odometry(object):
    """
    TODO ...
    """
    def __init__(self, camera):
        self.camera = camera
        self.resolution = self.camera.get_resolution()
        self.K = self.camera.K
        self.extrinsic = np.array(((1,0,0,0), (0,1,0,0), (0,0,1,0)))
        self.P = self.K @ self.extrinsic


    def project(self, pts):
        """
        TODO (ref: https://github.com/luigifreda/pyslam/blob/master/camera.py)
        """
        proj = self.camera.K @ pts.T
        z_coord = proj[-1] #Extract z-coordinate
        proj = proj[:2]/z_coord #Normalize to get correct img-coords (pixel units)
        return proj.T, z_coord


    def compute_transf_mat(self, pts1, pts2):
        """
        - Compute the camera pose from matched keypoints and the
          camera's intrinsic matrix.
        """
        #Essential matrix computed from 'K' and matched keypoints
        E, mask = cv.findEssentialMat(pts1, pts2, self.K)
        ret, R_, t_, mask = cv.recoverPose(E, pts1, pts2, self.K)
        #Solve for best solution
        R, t = self.optimize_pose(E, pts1, pts2)
        #Construct Transformation-mtx
        T = self.construct_transf_mat(R, np.squeeze(t))
        return T, (ret, R_, t_, mask)


    def optimize_pose(self, E, pts1, pts2):
        """
        - Determine the "best" solution for the Transformation matrix by
          reprojecting keypoints and solve for the solution with lowest error.
        """
        solutions_z_pos_sums = []
        relative_scales = []
        #Decompose 'E' into rotation-mtx and translation-vector
        R1, R2, t = cv.decomposeEssentialMat(E)
        t = np.squeeze(t)
        #Collect possible solutions
        solutions = [[R1, t], [R1, -t], [R2, t], [R2, -t]]
        #Solve for best solution
        for R, t in solutions:
            T = self.construct_transf_mat(R, t)
            #Compute Projection matrix
            P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
            #Perform triangulation
            hom_Q1 = cv.triangulatePoints(self.P, P, pts1.T, pts2.T)
            hom_Q2 = np.matmul(T, hom_Q1)
            #Unhomogenize
            Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
            #Sum points infront of camera
            sum_pos_z_Q1 = sum(Q1[2, :] > 0)
            sum_pos_z_Q2 = sum(Q2[2, :] > 0)
            #Calculate relative scale
            relative_scale = np.mean(
                np.linalg.norm(Q1.T[:-1] - Q1.T[1:], axis=-1) /
                np.linalg.norm(Q2.T[:-1] - Q2.T[1:], axis=-1))
            tot_sum_pos_z_Q1 = sum_pos_z_Q1 + sum_pos_z_Q2
            #Store results
            solutions_z_pos_sums.append(tot_sum_pos_z_Q1)
            relative_scales.append(relative_scale)
        #Select solution index with the most positive z-coords
        solution_idx = np.argmax(solutions_z_pos_sums)
        #Build solution
        solution = solutions[solution_idx]
        solution_relative_scale = relative_scales[solution_idx]
        R_solved, t_solved = solution
        t_solved = t_solved * relative_scale
        #Form Transformation/Projection matrix from solution
        T = self.construct_transf_mat(R_solved, t_solved)
        P = np.matmul(np.concatenate((self.K, np.zeros((3, 1))), axis=1), T)
        #Triangulate points again with solution
        hom_Q1 = cv.triangulatePoints(P, P, pts1.T, pts2.T)
        hom_Q2 = np.matmul(T, hom_Q1)
        #Unhomogenize
        Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
        Q2 = hom_Q2[:3, :] / hom_Q2[3, :]
        return [R_solved, t_solved]

    def undistort(self, img):


    @staticmethod
    def construct_transf_mat(R, t):
        """
        - Constructs Transformation matrix [4x4] from 'R' and 't', i.e the pose.
        """
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    @staticmethod
    def triangulate(pose1, pose2, pts1, pts2):
        """
        TODO ...
        """
        # [M1]
        ret = np.zeros((pts1.shape[0], 4))
        for i, p in enumerate(zip(pts1, pts2)):
            A = np.zeros((4,4))
            A[0] = p[0][0] * pose1[2] - pose1[0]
            A[1] = p[0][1] * pose1[2] - pose1[1]
            A[2] = p[1][0] * pose2[2] - pose2[0]
            A[3] = p[1][1] * pose2[2] - pose2[1]
            _, _, vt = np.linalg.svd(A)
            ret[i] = vt[3]
        # [M2]
        # ret = cv.triangulatePoints(pose1[:3], pose2[:3], pts1.T, pts2.T).T
        return ret

    @staticmethod
    def cart_to_hom(coords):
        """
        - Converts Cartesian coordinates to Homogeneous coordinates (2D->3D)
        """
        if coords.ndim == 1:
            return np.array([0])
        else:
            hom_coords = np.asarray(np.vstack([coords, np.ones(coords.shape[1])]))
            return hom_coords

    @staticmethod
    def homogenize(coords):
        """
        TODO ...
        """
        pass

    @staticmethod
    def add_ones(x):
        """
        TODO ...
        """
        return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)

    @staticmethod
    def normalize(Kinv, point):
        """
        TODO ...
        """
        return np.dot(Kinv, add_ones(point).T).T[:, 0:2]

    @staticmethod
    def denormalize(K, point):
        """
        TODO ...
        """
        ret = np.dot(K, np.array([point[0], point[1], 1.0]))
        ret /= ret[2]
        return int(round(ret[0])), int(round(ret[1]))
