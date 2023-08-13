import cv2 as cv


"""
Rewrite these according to the MonoVision classes instead
"""
class OpenCVLibrary(object):
    def __init__(self):
        pass

    def open_cap(self, cam_index):
        cap = cv.VideoCapture(cam_index)
        return cap

    def is_cap_open(self, cap):
        return cap.isOpened()

    def release_cap(self, cap):
        cap.release()

"""
Define Keywords: In your library code, define methods that wrap OpenCV
functionality and return values that Robot Framework can use for
assertions. In the example above, we have defined three keywords:
open_capture, is_capture_open, and release_capture.

Implement the "init.py" File: This file can be left empty. It's used
to indicate that the directory should be treated as a Python package.

Use Your Library in Robot Framework: Now you can create Robot
Framework test cases that utilize the keywords from your custom
library. Here's how a sample test case might look:

*** Settings ***
Library    MyOpenCVLibrary

*** Test Cases ***
Test Capture
    ${capture}    Open Capture    0
    ${is_open}    Is Capture Open    ${capture}
    Should Be True    ${is_open}
    Release Capture    ${capture}


"""
