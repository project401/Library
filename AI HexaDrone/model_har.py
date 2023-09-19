model_har.py
import cv2

# Load the Haar Cascade Classifier for face detection
face_cascade = cv2.CascadeClassifier('opencv/data/haarcascades/haarcascade_fullbody.xml')

# Open the video file or stream
cap = cv2.VideoCapture(0)

# Check if the video file or stream was successfully opened
if not cap.isOpened():
    print("Error opening video file or stream")
    exit()

# Loop through the frames in the video file or stream
while True:
    # Read a frame from the video file or stream
    ret, frame = cap.read()

    # Check if the frame was successfully read
    if not ret:
        print("Error reading frame")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame using the Haar Cascade Classifier
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3)

    # Draw bounding boxes around the detected faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Display the output frame with bounding boxes around the detected faces
    cv2.imshow('Face Detection', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video file or stream and destroy all windows
cap.release()
cv2.destroyAllWindows()
