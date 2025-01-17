import cv2
import numpy as np

def rectify_images(left_image_path, right_image_path):
    # Calibration data for the left camera
    left_camera_matrix = np.array([
        [1059.36028, 0.0, 970.83401],
        [0.0, 1060.96687, 560.9343],
        [0.0, 0.0, 1.0]
    ])
    left_distortion_coefficients = np.array([0.001269, 0.000482, -0.000113, -0.001515, 0.0])
    left_rectification_matrix = np.array([
        [0.99942511, 0.00193631, -0.03384815],
        [-0.00195502, 0.99999795, -0.00051971],
        [0.03384708, 0.00058558, 0.99942685]
    ])
    left_projection_matrix = np.array([
        [1096.89942, 0.0, 1031.42635, 0.0],
        [0.0, 1096.89942, 561.55067, 0.0],
        [0.0, 0.0, 1.0, 0.0]
    ])

    # Calibration data for the right camera
    right_camera_matrix = np.array([
        [1066.44441, 0.0, 974.78387],
        [0.0, 1067.43181, 562.47377],
        [0.0, 0.0, 1.0]
    ])
    right_distortion_coefficients = np.array([0.000737, -0.001881, 0.000007, -0.000908, 0.0])
    right_rectification_matrix = np.array([
        [0.99953987, 0.00197816, -0.03026782],
        [-0.00196143, 0.99999791, 0.00058248],
        [0.03026891, -0.00052284, 0.99954165]
    ])
    right_projection_matrix = np.array([
        [1096.89942, 0.0, 1031.42635, -132.35888],
        [0.0, 1096.89942, 561.55067, 0.0],
        [0.0, 0.0, 1.0, 0.0]
    ])

    # Load the images
    left_image = cv2.imread(left_image_path, cv2.IMREAD_COLOR)
    right_image = cv2.imread(right_image_path, cv2.IMREAD_COLOR)
    image_size = (left_image.shape[1], left_image.shape[0])  # (width, height)

    # Compute rectification maps for both cameras
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        left_camera_matrix,
        left_distortion_coefficients,
        left_rectification_matrix,
        left_projection_matrix,
        image_size,
        cv2.CV_32FC1
    )
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        right_camera_matrix,
        right_distortion_coefficients,
        right_rectification_matrix,
        right_projection_matrix,
        image_size,
        cv2.CV_32FC1
    )

    # Apply the rectification maps
    rectified_left = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    rectified_right = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # Combine the rectified images side by side
    combined_rectified = np.hstack((rectified_left, rectified_right))

    cv2.imshow("Left Rectified Image", rectified_left)
    cv2.imshow("Right Rectified Image", rectified_right)

    # Resize the combined image to make it smaller
    scale_percent = 50  # percent of original size
    width = int(combined_rectified.shape[1] * scale_percent / 100)
    height = int(combined_rectified.shape[0] * scale_percent / 100)
    dim = (width, height)
    resized_combined_rectified = cv2.resize(combined_rectified, dim, interpolation=cv2.INTER_AREA)

    # Display the combined and resized result
    cv2.imshow("Combined Rectified Images", resized_combined_rectified)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    # Paths to the left and right images
    left_image_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_left.jpg"
    right_image_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_right.jpg"

    rectify_images(left_image_path, right_image_path)
