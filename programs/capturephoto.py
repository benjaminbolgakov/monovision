import os
from monov.cap import Cap
import cv2 as cv

def capture_photo():
    W, H = 1920, 1080
    resolution = (W, H)
    cap = Cap(resolution)
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            cv.imshow('photo_capture', frame)
            if cv.waitKey(25) == ord('b'):
                cap.capture_img(frame)
            if cv.waitKey(25) == ord('q'):
                break

    cap.release()
    cv.destroyAllWindows()
