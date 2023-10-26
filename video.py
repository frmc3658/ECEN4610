import cv2

vid_capture = cv2.VideoCapture(0)
    

if not vid_capture.isOpened():
    print("Error opening webcam!")
else:
    fps = int(vid_capture.get(cv2.CAP_PROP_FPS))
    print("Frame Rate:", fps, "FPS")

    frame_count = vid_capture.get(cv2.CAP_PROP_FRAME_COUNT)
    print("Frame count:", frame_count)

    while vid_capture.isOpened():
        ret, frame = vid_capture.read()
        if ret:
            cv2.imshow('Frame', frame)
            k = cv2.waitKey(20)
            if k == 113:  # 'q' key to exit
                break
        else:
            break

    vid_capture.release()
    cv2.destroyAllWindows()
