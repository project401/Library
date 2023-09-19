import sys
sys.stderr = open('/dev/null', 'w')
import cv2
import caffe

# Load the model
model_def = 'Final-Models/Pedestrian-detection/deploy.prototxt'
model_weights = 'Final-Models/Pedestrian-detection/model.caffemodel'
net = caffe.Net(model_def, model_weights, caffe.TEST)

# Define preprocessing
transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})
transformer.set_transpose('data', (2,0,1))
transformer.set_raw_scale('data', 255)
transformer.set_channel_swap('data', (2,1,0))

# Start the video capture
cap = cv2.VideoCapture(0)

# Process the frames
while True:
    # Capture a frame
    ret, frame = cap.read()

    # Preprocess the frame
    image = transformer.preprocess('data', frame)
    net.blobs['data'].data[...] = image

    # Perform forward pass to get the output
    output = net.forward()['prob']

    # Do something with the output, e.g. display it on the frame
    label = output.argmax()
    cv2.putText(frame, str(label), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
    cv2.imshow('Frame', frame)

    # Exit on ESC
    if cv2.waitKey(1) == 27:
        break

# Release the video capture and close the windows
cap.release()
cv2.destroyAllWindows()
