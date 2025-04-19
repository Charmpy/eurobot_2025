import sys
import cv2
import numpy as np
import TIS

Tis = TIS.TIS()

# 39424442-v4l2 2048 1536 120/1 SinkFormats.BGRA True
# # The next line is for selecting a device, video format and frame rate.
# if not Tis.select_device():
#     quit(0)

Tis.open_device("39424442-v4l2", 2048, 1536, "120/1", TIS.SinkFormats.BGRA, False)
Tis.start_pipeline()  # Start the pipeline so the camera streams

print('Press Esc to stop')
lastkey = 0

cv2.namedWindow('Window')  # Create an OpenCV output window

kernel = np.ones((5, 5), np.uint8)  # Create a Kernel for OpenCV erode function

while lastkey != 27:
    if Tis.snap_image(1):  # Snap an image with one second timeout
        image = Tis.get_image()  # Get the image. It is a numpy array
        # image = cv2.erode(image, kernel, iterations=5)  # Example OpenCV image processing
        cv2.imshow('Window', image)  # Display the result

    lastkey = cv2.waitKey(10)

# Stop the pipeline and clean ip
Tis.stop_pipeline()
cv2.destroyAllWindows()
print('Program ends')