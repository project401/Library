import cv2
import picamera
import picamera.array

# Initialize the camera
camera = picamera.PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Start a stream with OpenCV
stream = picamera.array.PiRGBArray(camera)

# Create a window to display the video
cv2.namedWindow("Camera Video", cv2.WINDOW_NORMAL)

# Continuously capture frames from the camera and display them
for frame in camera.capture_continuous(stream, format="bgr", use_video_port=True):
    image = frame.array

    # Display the image in the window
    cv2.imshow("Camera Video", image)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    # Clear the stream for the next frame
    stream.seek(0)
    stream.truncate()

# Release the window and destroy all windows
cv2.destroyAllWindows()
