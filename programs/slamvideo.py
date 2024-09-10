def slam_video():
    vid_src = "../Videos/monovision/lab_gray/gray_comp.avi"
    W, H = 1920, 1080
    resolution = (W, H)
    camera = Camera(resolution, calib_src)
    #odo = Odometry(camera)
    cap = Cap(resolution, vid_src)
    slam = SLAM(camera)
    while cap.is_open():
        ret, frame = cap.read()
        if ret:
            #Start processing video
            slam.process_frame(frame)
            cap.display('slam_test', frame)

    cap.release()
