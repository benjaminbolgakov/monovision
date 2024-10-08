import os
import cv2 as cv
# import monov.cap
from monov import calibration
from monov.cap import Cap

def create_calibration_dir():
    srcdir = os.path.dirname(os.path.abspath(__file__))
    calibdir = os.path.join(srcdir, '../calibration/')
    while True:
        calibration_name = input("\nEnter name of calibration: ")
        targetdir = os.path.join(calibdir, calibration_name)
        # Verify calibration name doesn't already exist
        exists = os.path.isdir(targetdir)
        if exists:
            print(f"A calibration directory named '{calibration_name}' already exists, try again.")
        else:
            os.mkdir(targetdir)
            break

    # Create subdirs
    results_dir = os.path.join(targetdir, "results")
    src_dir = os.path.join(targetdir, "src")
    validation_dir = os.path.join(targetdir, "validation")
    os.mkdir(results_dir)
    os.mkdir(src_dir)
    os.mkdir(validation_dir)
    return targetdir

def capture_training_set(resolution, targetdir):
    print(resolution)
    cap = Cap(resolution)
    counter=0
    key = cv.waitKey(1)
    while counter <= 30 :
        ret, frame = cap.read()
        if ret:
            if key == ord('b'):
                fname = 'capture' + str(counter) + '.png'
                fpath = targetdir + "/src/" + fname
                cv.imwrite(fpath, frame)
                print(f"Saved image as: {fname}")
                counter += 1
    cap.release()
    cv.destroyAllWindows()

def capture():
    res = (1920, 1080)
    cap = Cap(res)
    counter=0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failure in receiving frame.")
            break

        running = cap.display('aruco_test', frame)
        if cv.waitKey(25) == ord('c'):
            fname = 'capture' + str(counter) + '.png'
            cv.imwrite(fname, frame)
            print(f"Saved image as: {fname}")
            counter+=1

    # Release cap and close window
    cap.release()
    cv.destroyAllWindows()

def calibrate_camera():
    """
     - Interactively creates two image sets:
       + One training set to calculate the camera parameters
       + One validation set to test the camera parameters on and assess precision.
     - Performs camera calibration and validation on the sets and writes results to
       a new calibration directory.
    """
    capture()
    # Acquire chessboard information
    print("\nEnter chessboard parameters")
    square_size = int(input("Square size(mm): "))
    x_squares = int(input("No. horizontal squares: "))
    y_squares = int(input("No. vertical squares: "))
    board_dimensions = (x_squares, y_squares)

    # Acquire desired camera resolution
    print("\nEnter desired camera resolution(W,H)")
    w_res = int(input("W: "))
    h_res = int(input("H: "))
    camera_resolution = (w_res, h_res)

    # Acquire a calibration name and setup directory structure
    calibration_dir = create_calibration_dir()

    # Capture training set
    capture_training_set(camera_resolution, calibration_dir)

    # Capture validation set

    # Perform calibration
    # calib_img_src = glob.glob("calibration/set_homecam/src/*")
    # valid_img_src = glob.glob("calibration/set_homecam/validation/*")
    # output_src = "calibration/set_homecam/results/"
    # square_size = 20 # chessboard square size (mm)
    # board = (8,5) # chessboard dimensions
    # resolution = (1920, 1080)
    # calibrate(calib_img_src, valid_img_src, resolution, square_size, board, output_src)
