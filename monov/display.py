import cv2 as cv

#### ------------- 2D Overlay Display ------------- ####
class OverlayDisplay(object):
    """
    TODO:
    1. Automatic scaling of font and automatic positioning
       of the text on the screen based on resolution+font-size
    """
    def __init__(self, resolution):
        """
        resolution = (W,H)
        """
        self.res = resolution
        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)
        self.font = cv.FONT_HERSHEY_PLAIN
        self.line = cv.LINE_AA
        self.font_scale_factor = self.scale_font()
        self.font_scale = 2*self.font_scale_factor
        self.font_scale_header = 3*self.font_scale_factor
        self.thickness = 1

    def scale_font(self):
        """
        Produce a scale factor for overlay text's font-size
        based on window-size.
        """
        standard_scale = 1
        scale_factor = min(self.res[0] / 1920, self.res[1] / 1080)
        font_scale = standard_scale * scale_factor
        text, header = 1*font_scale, 2*font_scale
        return font_scale

    def aruco_data(self, frame, markers, id_list):
        """
        - Draw aruco-detection-data onto frame.
        TODO:
        1. Automatic scaling of font and automatic positioning
           of the text on the screen based on resolution+font-size
        """
        id_closest = markers[0][0]
        distance_closest = markers[0][1]
        bearing_closest = markers[0][2]
        # Detection status
        frame = cv.putText(frame, "Detection Status: ", (10,30),
            self.font, self.font_scale_header, self.red, self.thickness, self.line)
        frame = cv.putText(frame, "ID's: " + str(id_list), (10,70),
            self.font, self.font_scale, self.red, self.thickness, self.line)
        frame = cv.putText(frame, "Closest marker: " + str(id_closest), (10,105),
            self.font, self.font_scale, self.red, self.thickness, self.line)
        frame = cv.putText(frame, "Distance: " + str(distance_closest), (10,140),
            self.font, self.font_scale, self.red, self.thickness, self.line)
        frame = cv.putText(frame, "Bearing: " + str(bearing_closest), (10,175),
            self.font, self.font_scale, self.red, self.thickness, self.line)

#### ------------- 2D SLAM Display ------------- ####
# import pygame
# #pygame.init()
# from pygame.locals import DOUBLEBUF
# class Display2D(object):
#     def __init__(self, W=1920, H=1080):
#         pygame.init()
#         pygame.display.set_caption('SLAM')
#         self.screen = pygame.display.set_mode((W,H), DOUBLEBUF)
#         self.surface = pygame.Surface(self.screen.get_size()).convert()

#     def quit(self):
#         pygame.display.quit()
#         pygame.quit()

#     def draw(self, img):
#         pygame.surfarray.blit_array(self.surface, img.swapaxes(0,1)[:, :, [0,1,2]])
#         self.screen.blit(self.surface, (0, 0))
#         pygame.display.flip()
