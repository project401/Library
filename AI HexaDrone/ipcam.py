ipcam.py
import cv2

# IP camera URL
url = "rtsp://admin:project401R&D@192.168.0.16"

# Set desired resolution
width, height = 640, 480

# Open the IP camera stream with reduced buffer size
cap = cv2.VideoCapture(url)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    # Read the current frame from the stream
    ret, frame = cap.read()

    # Display the frame in a window
    cv2.imshow('IP Camera Feed', frame)

    # Wait for a key press for a short duration
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
