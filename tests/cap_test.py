import cv2 as cv
from cap import Cap

#### Helpers/Constants ####
highres = (1920, 1080)
lowres = (640, 480)


#### Tests ####
"""
NOTE:
- 'cap.set()' will return True if the property is supported
  by backend used by the VideoCapture instance.
- According to OpenCV documentation, there's no guarantee of
  the 'set' command *actually* working even though it returns
  true.
"""

def cap_props_fps():
    """
    - Tests setting the FPS-property.
    """
    cap = Cap(lowres)
    fps = 30
    cap_fps_set = cap.set(cv.CAP_PROP_FPS, fps)
    assert cap_fps_set == True
    cap_fps_get = cap.get(cv.CAP_PROP_FPS)
    assert cap_fps_get == fps

def cap_props_buffer():
    """
    - Tests setting the buffer-size property.
    """
    cap = Cap(lowres)
    buffer_size = 3
    cap_buffer_set = cap.set(cv.CAP_PROP_BUFFERSIZE, buffer_size)
    assert cap_buffer_set == True
    cap_buffer_get = cap.get(cv.CAP_PROP_BUFFERSIZE)
    assert cap_buffer_get == buffer_size

def cap_props_dialog():
    """
    - Tests enabling the pop-up settings dialog.
    """
    cap = Cap(highres)
    enabled = 1
    cap_dialog_set = cap.set(cv.CAP_PROP_SETTINGS, enabled)
    assert cap_dialog_set == True
    cap_dialog_get = cap.get(cv.CAP_PROP_SETTINGS)
    assert cap_dialog_get == enabled

def cap_props_resolution():
    """
    - Test if VideoCapture property settings was successful
      on current system. Creating the 'cap' with given frame_size
      should set the VideoCapture's width/height property accordingly
      but might fail due to issues in the system layer chain.
      (CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT)
    """
    cap = Cap(highres)
    assert cap.get_resolution() == highres
