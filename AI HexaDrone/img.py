import picamera

# Initialize the camera
camera = picamera.PiCamera()

# Set the camera resolution
camera.resolution = (640, 480)

# Take the picture
camera.capture('image.jpg')

# Close the camera
camera.close()
