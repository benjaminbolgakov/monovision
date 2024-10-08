

def aruco_vid_testing():
    vid_src = "media/test_vid.avi"
    marker_size = 156 # aruco marker size (mm)
    resolution = (1920, 1080)
    camera = Camera(resolution, calib_src)
    #odo = Odometry(resolution)
    cap = Cap(resolution, vid_src)
    aruco = Aruco(camera, marker_size)
    display = OverlayDisplay(resolution)
    #Begin process video
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            markers, id_list = aruco.scan(frame, True) # [ [id, distance, bearing], ..] ]
            print(markers)
            display.aruco_data(frame, markers, id_list)
            cap.display('aruco_test', frame)

    cap.release()
