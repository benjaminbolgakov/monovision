import os
import platform
import cv2 as cv
from ._utils import file_checker, fourcc_to_codec, get_backend_name

#VideoCapture backend identifiers
backends = {
    "Windows": [cv.CAP_DSHOW, cv.CAP_MSMF, cv.CAP_FFMPEG, cv.CAP_ANY],
    "Linux": [cv.CAP_V4L2, cv.CAP_V4L, cv.CAP_FFMPEG, cv.CAP_GSTREAMER, cv.CAP_DSHOW, cv.CAP_ANY]
}
class Cap(object):
    """
    - System layer chain:
    VideoCapture -> Backend -> OS -> Device Driver -> Device Hardware
    TODO:
    1. Enable creating a VideoWriter with this class to enable
       recording the feed/video.
    2. Move '.display' to display.py instead?
    """
    def __init__(self, resolution, vid_src=None, fps=30):
        """
        - Creates an OpenCV VideoCapture object and set's it's
          properties based on the current running OS.
        - If 'vid_src' is None the cap will operate from a
          live camera feed instead.
          ====================================================
          resolution = Resolution (W,H)
          vid_src = Video file to render using cap.
        """
        self.resolution = resolution
        self.fps = fps
        self.os_ = platform.system()
        self.cap_api = self.config_api(self.os_)
        self.cap = self.create_cap(vid_src)
        self.cap_w, self.cap_h = self.get_resolution()

    def create_cap(self, vid_src, cam_id=0, buffer_size=4):
        """
        TODO ...
        """
        print("============= Creating a Cap =============")
        if vid_src is None:
            cap = cv.VideoCapture(cam_id, self.cap_api[0]) #V4L2 (200)
        else:
            cap = cv.VideoCapture(vid_src, self.cap_api[1]) #FFMPEG (1900)
        #Setup cap properties based on current OS
        if cap.isOpened():
            self.set_cap_props(cap, self.os_, self.resolution, buffer_size)
            self.cap_w = cap.get(cv.CAP_PROP_FRAME_WIDTH)
            self.cap_h = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        else:
            print("Cap failed to open.")
        print("CAP: " + str(cap))
        return cap

    def create_recorder(self, f_path):
        """
        - Creates a VideoWriter object based on current running OS.
        TODO:
        1. In case '_', is XVID optimal?
        """
        #Set compression codec based on OS
        match os_:
            case "Windows":
                fourcc = cv.VideoWriter_fourcc(*'MJPG')
            case "Linux":
                fourcc = cv.VideoWriter_fourcc(*'XVID')
            case _:
                fourcc = cv.VideoWriter_fourcc(*'XVID') #Is this optimal?
        #Validate file path and name
        f_path = file_checker(f_path, "monovision_video.avi")
        recorder = cv.VideoWriter(f_path, fourcc, self.fps, self.resolution)
        return recorder

    def display(self, window_name, frame):
        try:
            cv.resize(frame, self.resolution)
            cv.imshow(window_name, frame)
            if cv.waitKey(25) == ord('q'):

        except TypeError:
            print("Invalid window name")


    def config_api(self, os_):
        """
        TODO
        """
        match os_:
            case "Windows":
                print("\n *Windows system detected - Setting up DirectShow backend.")
                api_feed = cv.CAP_DSHOW
                api_stream = cv.CAP_FFMPEG #Maybe change to DSHOW?
            case "Linux":
                print("\n *Linux system detected - Setting up V4L(2) backend.")
                api_feed = cv.CAP_V4L2
                api_stream = cv.CAP_FFMPEG
            case _:
                print("\n *No system identifer found - Autodetecting backend.")
                api_feed = cv.CAP_ANY
                api_stream = cv.CAP_FFMPEG

        return (api_feed, api_stream)

    def set_cap_props(self, cap, os_, resolution, buffer_size):
        """
        Applies required OS-specific props to the 'cap' object.
        *Logitech C930e: 1920x1080 @ 30fps*
        Windows requires special treatment in order to capture
        MJPEG compressed video from the feed, for some reason...
        Ref: www.fourcc.org
        NOTE:
        - Must set FOURCC before FRAME_W/H properties, otherwise no effect.
        - Buffer size is only supported by DC1394 v 2.x backend.
        =======================================================================
        cap = VideoCapture object.
        os_ = Detected OS
        resolution = Frame resolution (W, H)
        buffer_size = Number of frames internal buffer can hold
        """
        # Define the codec for output video
        if self.os_ == 'Windows': #fourcc = FFMPEG or MSWF or DSHOW
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m','j','p','g'))
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M','J','P','G'))
        elif self.os_ == 'Linux': #fourcc = FFMPEG
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        else:
            cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        #Empty list for the results of setting props
        prop_set_results = [None for _ in range(4)]
        print(" *Attempting to configure system with the following properties:")
        print(" -FPS: %d \n -Resolution: %r \n -Buffer size: %d \n -Codec: MJPG \n" %
              (self.fps, resolution, buffer_size))
        #Try setting props
        prop_set_results[0] = cap.set(cv.CAP_PROP_FPS, self.fps)
        prop_set_results[1] = cap.set(cv.CAP_PROP_FRAME_WIDTH, resolution[0])
        prop_set_results[2] = cap.set(cv.CAP_PROP_FRAME_HEIGHT, resolution[1])
        prop_set_results[3] = cap.set(cv.CAP_PROP_BUFFERSIZE, buffer_size)
        #Insert descriptors of prop results: ["fps", False] etc..
        for prop in prop_set_results:
            if prop == False:
                print("Failed setting prop")
            else:
                print("Succeded setting prop")
        self.print_cap_props(cap)

    def get_cap_props(self, cap):
        fps = cap.get(cv.CAP_PROP_FPS)
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        buffer_size = cap.get(cv.CAP_PROP_BUFFERSIZE)
        fourcc = fourcc_to_codec(cap.get(cv.CAP_PROP_FOURCC))
        backend = get_backend_name(cap.get(cv.CAP_PROP_BACKEND))
        resolution = (width, height)
        return (fps, resolution, buffer_size, fourcc, backend)

    def print_cap_props(self, cap):
        """
        - Prints the cap properties
        """
        config = self.get_cap_props(cap)
        print("\n=System configuration= ")
        print(" -FPS: %d \n -Resolution: %r \n -Buffer size: %d \n -Codec: %s \n -Backend: %s \n" %
              (config[0], config[1], config[2], config[3], config[4]))
        # print("FPS: ", cap.get(cv.CAP_PROP_FPS))
        # print("FRAME_W: ", cap.get(cv.CAP_PROP_FRAME_WIDTH))
        # print("FRAME_H: ", cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        # print("BUFFER_S: ", cap.get(cv.CAP_PROP_BUFFERSIZE))
        # print("FOURCC: ", cap.get(cv.CAP_PROP_FOURCC))
        # print("BACKEND: ", cap.get(cv.CAP_PROP_BACKEND))

    def print_system_config(self):
        """
        - Prints various information about the setup of the system
        """
        print("System Information: \n")
        print("OS: ", self.os_, "\n")
        print("Frame Size: ", self.resolution)


    def is_open(self):
        return self.cap.isOpened()

    def get_resolution(self):
        cap_w = self.cap.get(cv.CAP_PROP_FRAME_WIDTH)
        cap_h = self.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        return (cap_w, cap_h)

    def release(self):
        # if a VideoWriter is active, release it as well...
        self.cap.release()

    def read(self):
        ret, frame = self.cap.read()
        return ret, frame
