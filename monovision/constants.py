
backends = {
    "Windows": [cv.CAP_DSHOW, cv.CAP_MSMF, cv.CAP_FFMPEG, cv.CAP_ANY],
    "Linux": [cv.CAP_V4L2, cv.CAP_V4L, cv.CAP_FFMPEG, cv.CAP_GSTREAMER, cv.CAP_DSHOW, cv.CAP_ANY]
}

opencv_video_backends = {
    "CAP_ANY": 0,  # Automatically choose a backend (default)
    "V4L": 200,       # Video for Linux (V4L/V4L2)
    "V4L2": 200,      # Video for Linux 2 (V4L2)
    "VFW": 200,       # Video for Windows (VFW)
    "DSHOW": 700,     # DirectShow (via the videoInput library)
    "CV_MSMF": 1400,  # Microsoft Media Foundation (Windows)
    "GSTREAMER": 1800,# GStreamer (Linux, macOS, and Windows)
    "GST": 1800,      # GStreamer (alias for GSTREAMER)
    "FFMPEG": 1900,   # FFmpeg
    "CV_IMAGES": 2000,# OpenCV Images (for testing)
    "ARAVIS": 2100,   # Aravis GigE Vision (Linux)
    "OPENCV_MJPEG": 2200,  # OpenCV MJPEG codec (for testing)
}
