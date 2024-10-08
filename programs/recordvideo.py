# def record_video():
#     W, H = 1920, 1080
#     resolution = (W, H)
#     cap = Cap(resolution)
#     recorder = cap.create_recorder("test_sample")
#     while cap.is_open():
#         ret, frame = cap.read()
#         if ret:
#             cv.imshow('recording', frame)
#             recorder.write(frame)
#             if cv.waitKey(25) == ord('q'):
#                 break

#     cap.release()
#     recorder.release()
#     cv.destroyAllWindows()
