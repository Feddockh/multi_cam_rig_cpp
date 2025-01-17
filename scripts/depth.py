import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load left and right images
left_image = cv2.imread("/home/hayden/data/director_sync/1736894279_398142322_zed_left.jpg", cv2.IMREAD_GRAYSCALE)
right_image = cv2.imread("/home/hayden/data/director_sync/1736894279_398142322_zed_right.jpg", cv2.IMREAD_GRAYSCALE)

# Create a stereo block matcher object
stereo = cv2.StereoBM_create(numDisparities=64, blockSize=15)

# Compute the disparity map
disparity = stereo.compute(left_image, right_image)

# Normalize the disparity map for visualization
disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

# Visualize the disparity map
plt.figure(figsize=(10, 5))
plt.title("Disparity Map")
plt.imshow(disparity_normalized, cmap="jet")
plt.colorbar(label="Disparity")
plt.show()

# Camera parameters
focal_length = 1096.89942
baseline = 0.1207

# Avoid dividing by zero
disparity[disparity <= 0] = 0.1

# Compute depth map
depth_map = (focal_length * baseline) / disparity

# Visualize the depth map
plt.figure(figsize=(10, 5))
plt.title("Depth Map")
plt.imshow(depth_map, cmap="jet")
plt.colorbar(label="Depth (meters)")
plt.show()
