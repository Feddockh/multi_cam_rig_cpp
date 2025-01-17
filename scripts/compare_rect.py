import cv2
import numpy as np

def compare_rectification(image1_path, image2_path, points1, points2):
    # Load images
    image1 = cv2.imread(image1_path)
    image2 = cv2.imread(image2_path)

    # Convert points to numpy arrays
    points1 = np.array(points1, dtype=np.float32)
    points2 = np.array(points2, dtype=np.float32)

    # Step 1: Rectify using stereoRectifyUncalibrated
    print("Computing uncalibrated rectification...")
    F, mask = cv2.findFundamentalMat(points1, points2, cv2.FM_RANSAC)
    retval, h1_uncalib, h2_uncalib = cv2.stereoRectifyUncalibrated(points1, points2, F, image1.shape[:2])

    # Step 2: Rectify using calibration data
    print("Rectifying using calibration data...")
    
    h1_calib = np.array([
        [0.99942511, 0.00193631, -0.03384815],
        [-0.00195502, 0.99999795, -0.00051971],
        [0.03384708, 0.00058558, 0.99942685]
    ])
    h2_calib = np.array([
        [0.99953987, 0.00197816, -0.03026782],
        [-0.00196143, 0.99999791, 0.00058248],
        [0.03026891, -0.00052284, 0.99954165]
    ])

    # Warp images using uncalibrated rectification
    print("Warping images using uncalibrated rectification matrices...")
    height, width, _ = image1.shape
    rectified1_uncalib = cv2.warpPerspective(image1, h1_uncalib, (width, height))
    rectified2_uncalib = cv2.warpPerspective(image2, h2_uncalib, (width, height))

    # Warp images using calibration rectification matrices
    print("Warping images using calibration rectification matrices...")
    rectified1_calib = cv2.warpPerspective(image1, h1_calib, (width, height))
    rectified2_calib = cv2.warpPerspective(image2, h2_calib, (width, height))

    # Combine results for visualization
    combined_uncalib = np.hstack((rectified1_uncalib, rectified2_uncalib))
    combined_calib = np.hstack((rectified1_calib, rectified2_calib))

    # Display the results
    cv2.imshow("Uncalibrated Rectification", combined_uncalib)
    cv2.imshow("Calibrated Rectification", combined_calib)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Example usage
if __name__ == "__main__":
    
    image1_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_left.jpg"
    image2_path = "/home/hayden/data/director_sync/1736894279_398142322_zed_right.jpg"

    # Corresponding points (example data)
    points1 = [(980, 538), (780, 328), (1534, 208), (854, 184), (430, 484), (566, 978),
               (492, 956), (484, 846), (1696, 624), (1428, 766), (1566, 906), (1838, 932),
               (470, 248), (1574, 226), (1540, 522), (1358, 564)]
    points2 = [(630, 542), (582, 328), (1486, 210), (656, 184), (144, 484), (362, 976),
               (274, 956), (252, 844), (1416, 624), (1174, 768), (1468, 906), (1748, 932),
               (298, 248), (1476, 224), (1490, 516), (1118, 556)]

    compare_rectification(image1_path, image2_path, points1, points2)
