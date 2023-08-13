import pickle
import numpy as np

"""
NOTE:
1. Camera pose verification:
   Determinant of rotation matrix is = 1.
   So, if det(R) = -1, camera pose is correct, i.e:
   -C -> C and -R -> R

"""

class Camera(object):
    """
    - Represents the camera/agent moving about in the world.
    ========================================================
    TODO:
    - Implement ability to use uncalibrated camera (see TwitchSLAM), hence the
      'hasattr' conditioning on the properties.
    - Consider the 'property' variables. Should 'K' be initialized instead?
    """
    def __init__(self, resolution, calibration_src):
        self.resolution = resolution
        self.model = self.extract_model(calibration_src)
        self.mtx = self.model[0]
        self.dist = self.model[1]
        self.extrinsic = np.array(((1,0,0,0), (0,1,0,0), (0,0,1,0)))
        self.cur_pose = np.array(
            [[1., 0., 0., 0.],
            [0., 1., 0., 0.],
            [0., 0., 1., 0.]])
        self.position = (None, None) #(x,y)

    def update_pose(self, T):
        #assert T.ndim == something..
        self.cur_pose = self.cur_pose @ T
        #Update position
        self.position = (self.cur_pose[0, 3], self.cur_pose[2, 3])

    #OBSOLETE?
    def set_position(self, heading, distance):
        self.x += np.cos(heading) * distance
        self.y += np.sin(heading) * distance

    #OBSOLETE?
    def add_landmark(self, landmark):
        self.landmarks.append(landmark)

    def extract_model(self, calib_src):
        """
        - Extracts calibration parameters.
        Params
        ------
        calib_src: calibration file location.
        model: (mtx, dist, rvec, tvec, mtx_n)
        """
        #Extract calibration-components
        with open(calib_src, 'rb') as calibration:
            ret, mtx, dist, rvec, tvec, mtx_n = pickle.load(calibration)
        #Round values of mtx components
        mtx = np.round(mtx).astype(int)
        mtx_n = np.round(mtx_n).astype(int)
        model = (mtx, dist, rvec, tvec, mtx_n)
        return model

    def extract_intrinsics(self, model):
        """
        - Extracts intrinsics from camera model
        Params
        ------
        model: (mtx, dist, rvec, tvec, mtx_n)
        K: intrinsic matrix (3x3 numpy array)
        """
        mtx = model[0] #3x3
        fx = int(round(mtx[0,0]))
        fy = int(round(mtx[1,1]))
        cx = int(round(mtx[0,2]))
        cy = int(round(mtx[1,2]))
        # Form the intrinsic camera matrix
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]])
        return K

    def extract_projection_mtx(self, model):
        """
        - Extracts intrinsics from camera model
        Params
        ------
        model: camera model (mtx, dist, rvec, tvec, mtx_n)
        P: Projection matrix (3x4 numpy array)
        """
        mtx = model[0] #3x3
        P = np.hstack((mtx, np.zeros((3,1)))) #3x4
        return P

    def get_resolution(self):
        return self.resolution

    @property
    def P(self):
        if not hasattr(self, '_P'):
            self._P = self.extract_projection_mtx(self.model)
        return self._P

    @property
    def K(self):
        if not hasattr(self, '_K'):
            self._K = self.extract_intrinsics(self.model)
        return self._K

    @property
    def Kinv(self):
        if not hasattr(self, '_Kinv'):
            self._Kinv = np.linalg.inv(self.K)
        return self._Kinv
