import cv2 as cv
from cv2 import aruco
import numpy as np
from vision import Camera

class Aruco(Camera):
    """
    - Sub Class of base class 'Camera'
    ===================================
    marker_size: lenght of the entire side of aruco marker
    TODO ...
    """
    #### - Class Variables - ####
    def_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) #Default aruco dictionary

    def __init__(self, camera, marker_size, a_dict=def_dict):
        """
        TODO ...
        """
        self.a_dict = a_dict
        self.mtx = camera.mtx.astype(float) #solvePnP need floating point values
        self.dist = camera.dist
        self.params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.a_dict, self.params)
        self.marker_size = marker_size #mm
        self.marker_objpts = np.array([[-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
        self.objpoints = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0]], dtype=np.float32)
        self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

    def gen_aruco(self, a_id, a_size):
        """
        Generate an Aruco marker.
        """
        fig = aruco.generateImageMarker(self.a_dict, a_id, a_size)
        plt.imshow(fig, cmap=mpl.cm.gray, interpolation="nearest")
        plt.axis("off")
        plt.show()

    def gen_aruco_set(self, a_size, nx, ny):
        """
        Generate a set of Aruco marker
        """
        fig = plt.figure()
        for i in range(1, nx*ny+1):
            ax = fig.add_subplot(ny,nx, i)
            img = aruco.generateImageMarker(self.a_dict, i, a_size)
            plt.imshow(img, cmap=mpl.cm.gray, interpolation="nearest")
            ax.axis("off")
        plt.show()

    def detect(self, frame):
        """
        Run detection-algorithm on current frame
        """
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        aruco.drawDetectedMarkers(frame, corners, ids)
        return corners, ids

    def scan(self, frame, sort=False, flags=None):
        """
        - Scan for aruco markers. Calculate each markers distance
          and bearing to the camera. Enable 'sort' param to sort
          and return list of detected markers in order by distance.
        ===========================================================
        marker = [id, distance, bearing]
        """
        #Scan for aruco markers
        corners, ids = self.detect(frame)
        if ids is not None:
            #Init empty marker list
            marker_list = [[0 for x in range(3)] for y in range(len(corners))]
            if len(corners) > 0:
                for i in range(len(corners)):
                    #Estimate pose of detected markers
                    rvec, tvec, imgpts = self.estimate_pose(frame, corners[i], ids, flags)
                    #Calculate distance & bearing to markers
                    distance = self.estimate_distance(rvec, tvec)
                    bearing = self.estimate_bearing(tvec, rvec)
                    #Package marker parameters with their ID
                    marker_id = ids[i][0]
                    marker = [marker_id, distance, bearing]
                    #Store detected marker and distance to it
                    marker_list[i] = marker

                # if sort: #Sort markers based on distance
                marker_list.sort(key=lambda marker: marker[1])

            id_list = []
            #Seperate list of all found ID's
            id_list = [marker_list[x][0] for x in range(len(marker_list))]
            return marker_list, id_list
        else:
            return [[404, 404, 404]], [404]

    def sort_markers(self, marker_list):
        """
        - Sort a list of markers by their distance to the camera
        =========================================================
        marker_list = [[id, distance, bearing], ...]
        """
        if marker_list is not None and len(marker_list) > 0:
            #Sort markers based on distance
            marker_list.sort(key=lambda marker: marker[1])
            return marker_list
        else:
            return [[404, 404, 404]]

    def estimate_pose(self, frame, corners, ids, flags=None):
        """
        Estimate pose of Aruco marker
        """
        axis_length = self.marker_size / 2
        if len(corners) > 0:
            ids_f = ids.copy()
            ids_f= ids.flatten() #Why do this? (NicolaiNielsen)
            ret, rvecs, tvecs = cv.solvePnP(self.marker_objpts, corners[0], self.mtx, self.dist, False, flags)
            imgpts, jac = cv.projectPoints(self.marker_objpts, rvecs, tvecs, self.mtx, self.dist)
            #self.util.project(frame, imgpts)
            cv.drawFrameAxes(frame, self.mtx, self.dist, rvecs, tvecs, axis_length)

        return rvecs, tvecs, imgpts


    def euclideanDistanceOfTvecs(tvec1, tvec2):
        return math.sqrt(math.pow(tvec1[0]-tvec2[0], 2) +
                         math.pow(tvec1[1]-tvec2[1], 2) +
                         math.pow(tvec1[2]-tvec2[2], 2))

    def estimate_distance(self, rvec, tvec):
        """
        TVEC (translation vector) is in the same unit as provided in 'marker_size'
        in the 'pose' function and comes in the format [x,y,z], where 'z' from 'tvec'
        will be the distance from camera to marker center.
        tvec:  [[ -25.46647829], [-126.21458748], [ 743.23759615]]
        """
        #distance = abs(tvec[2][0])
        #distance = np.linalg.norm(tvec)
        distance = np.sqrt(tvec[0][0] ** 2 + tvec[1][0] ** 2 + tvec[2][0] ** 2) #Pythagoras
        return distance

    def estimate_bearing(self, tvec, rvec):
        """
        Calculate the bearing angle
        tvec = [[ 119.29900914], [-134.24204244], [ 873.6529513 ]]

        """
        R_marker_to_cam, _ = cv.Rodrigues(rvec)
        R_cam_to_marker = cv.transpose(R_marker_to_cam)
        t_cam_to_marker = -R_cam_to_marker.dot(tvec)
        p_cam_in_marker_frame = -R_cam_to_marker.dot(tvec)
        # b = np.arctan2(tvec[0][0], tvec[2][0])
        bearing_rad = np.arctan2(p_cam_in_marker_frame[1], p_cam_in_marker_frame[0])
        bearing_deg = np.rad2deg(bearing_rad)
        return bearing_deg[0]

# class Aruco(object):
#     """
#     marker_size: lenght of the entire side of aruco marker
#     """
#     #### - Class Variables - ####
#     def_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) #Default aruco dictionary

#     def __init__(self, mtx, dist, marker_size, a_dict=def_dict):
#         self.a_dict = a_dict
#         self.mtx = mtx
#         self.dist = dist
#         self.params = aruco.DetectorParameters()
#         self.detector = aruco.ArucoDetector(self.a_dict, self.params)
#         self.marker_size = marker_size #mm
#         self.marker_objpts = np.array([[-self.marker_size/2,  self.marker_size/2, 0],
#             [ self.marker_size/2,  self.marker_size/2, 0],
#             [ self.marker_size/2, -self.marker_size/2, 0],
#             [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
#         self.objpoints = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0]], dtype=np.float32)
#         self.criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#         self.axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

#     def gen_aruco(self, a_id, a_size):
#         """
#         Generate an Aruco marker.
#         """
#         fig = aruco.generateImageMarker(self.a_dict, a_id, a_size)
#         plt.imshow(fig, cmap=mpl.cm.gray, interpolation="nearest")
#         plt.axis("off")
#         plt.show()

#     def gen_aruco_set(self, a_size, nx, ny):
#         """
#         Generate a set of Aruco marker
#         """
#         fig = plt.figure()
#         for i in range(1, nx*ny+1):
#             ax = fig.add_subplot(ny,nx, i)
#             img = aruco.generateImageMarker(self.a_dict, i, a_size)
#             plt.imshow(img, cmap=mpl.cm.gray, interpolation="nearest")
#             ax.axis("off")
#         plt.show()

#     def scan(self, frame, flags=None):
#         """
#         Takes input-feed, detects markers, packages them in a sorted order
#         based on distance from it (closest markers at [0]).
#         marker: [ [[id]], [distance], [bearing] ]
#         - Investigate if this is valuable to do before detection:
#         gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#         cv.undistort(gray....)
#         Try adding a condition of having a marker being detected for x amount of time
#         before pushing it to the detected markers list, to further filter out noise.
#         """
#         #Scan for aruco markers
#         corners, ids = self.detect(frame)
#         if ids is not None:
#             #Marker = [id, distance, bearing]
#             #Init empty marker list = [[0,0,0],[0,0,0],..]
#             marker_list = [[0 for x in range(3)] for y in range(len(corners))]
#             if len(corners) > 0:
#                 for i in range(len(corners)):
#                     #Estimate pose of detected markers
#                     rvec, tvec, imgpts = self.pose(frame, corners[i],ids, self.mtx, self.dist, flags)
#                     #Calculate distance to markers
#                     d1, d2, d5= self.distance(rvec, tvec)
#                     bearing = self.bearing(tvec, rvec)
#                     #Compile marker parameters
#                     marker = [ids[i][0], d1, bearing]
#                     #Store detected marker and distance to it
#                     marker_list[i] = marker

#             #Sort markers based on distance
#             marker_list.sort(key=lambda marker: marker[1])
#             id_list = []
#             #Store ID's of all detected markers
#             id_list = [marker_list[x][0] for x in range(len(marker_list))]
#             return marker_list, id_list
#         else:
#             return [[404, 404, 404]], [404]

#     def detect(self, frame):
#         """
#         Run detection-algorithm on current frame
#         """
#         gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#         corners, ids, _ = self.detector.detectMarkers(gray)
#         aruco.drawDetectedMarkers(frame, corners, ids)
#         return corners, ids

#     def pose(self, frame, corners, ids, mtx, dist, flags):
#         """
#         Estimate pose of Aruco marker
#         """
#         axis_length = self.marker_size / 2
#         if len(corners) > 0:
#             ids_f = ids.copy()
#             ids_f= ids.flatten() #Why do this? (NicolaiNielsen)
#             ret, rvecs, tvecs = cv.solvePnP(self.marker_objpts, corners[0], mtx, dist, False, flags)
#             imgpts, jac = cv.projectPoints(self.marker_objpts, rvecs, tvecs, mtx, dist)
#             #self.util.project(frame, imgpts)
#             cv.drawFrameAxes(frame, mtx, dist, rvecs, tvecs, axis_length)

#         return rvecs, tvecs, imgpts


#     def euclideanDistanceOfTvecs(tvec1, tvec2):
#         return math.sqrt(math.pow(tvec1[0]-tvec2[0], 2) +
#                          math.pow(tvec1[1]-tvec2[1], 2) +
#                          math.pow(tvec1[2]-tvec2[2], 2))

#     def distance(self, rvec, tvec):
#         """
#         TVEC (translation vector) is in the same unit as provided in 'marker_size'
#         in the 'pose' function and comes in the format [x,y,z], where 'z' from 'tvec'
#         will be the distance from camera to marker center.
#         tvec:  [[ -25.46647829], [-126.21458748], [ 743.23759615]]
#         """
#         d1 = abs(tvec[2][0])
#         d2 = np.linalg.norm(tvec)
#         d3 = np.sqrt(tvec[0][0] ** 2 + tvec[1][0] ** 2 + tvec[2][0] ** 2) #Pythagoras
#         return (d1, d2, d3)

#     def bearing(self, tvec, rvec):
#         """
#         Calculate the bearing angle
#         tvec = [[ 119.29900914], [-134.24204244], [ 873.6529513 ]]

#         """
#         R_marker_to_cam, _ = cv.Rodrigues(rvec)
#         R_cam_to_marker = cv.transpose(R_marker_to_cam)
#         t_cam_to_marker = -R_cam_to_marker.dot(tvec)
#         p_cam_in_marker_frame = -R_cam_to_marker.dot(tvec)
#         bb = np.arctan2(p_cam_in_marker_frame[1], p_cam_in_marker_frame[0])
#         b = np.rad2deg(bb)
#         #b = np.arctan2(tvec[0][0], tvec[2][0])
#         return b
