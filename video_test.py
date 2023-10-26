import cv2

# Create a VideoCapture object to access the camera (use 0 for the default camera)
cap = cv2.VideoCapture(0)

# Check if the camera was opened successfully
if not cap.isOpened():
    print("Error opening the camera.")
else:
    while True:
        # Read a frame from the camera.
        ret, frame = cap.read()

        if not ret:
            print("Error reading a frame.")
            break

        # Display the frame in a window
        cv2.imshow('Camera Feed', frame)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close the OpenCV window
    cap.release()
    cv2.destroyAllWindows()
