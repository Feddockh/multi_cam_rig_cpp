# Hayden Feddock
# 3/21/2025

import os
import cv2
import numpy as np
from typing import List, Dict, Tuple
from scipy.optimize import least_squares
import yaml


EXTENSION = ".png"
CALIBRATION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration_files")

class Camera:
    def __init__(self, name: str):
        self.name = name
        self.width: int = 0
        self.height: int = 0
        self.error: float = 0.0
        self.camera_matrix: np.ndarray = np.zeros((3, 3), dtype=np.float32)
        self.dist_coeffs: np.ndarray = np.zeros((5,), dtype=np.float32)
        self.rectification_matrix: np.ndarray = np.eye(3, dtype=np.float32)
        self.projection_matrix: np.ndarray = np.zeros((3, 4), dtype=np.float32)
        self.transforms: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}

    def load_params(self):
        """
        Load the camera parameters from a YAML file.
        """
        yaml_file = os.path.join(CALIBRATION_DIR, f"{self.name}.yaml")
        if not os.path.exists(yaml_file):
            raise FileNotFoundError(f"Calibration file {yaml_file} not found.")
        with open(yaml_file, 'r') as f:
            calib_data = yaml.safe_load(f)
        self.width = int(calib_data['image_width'])
        self.height = int(calib_data['image_height'])
        self.camera_matrix = np.array(calib_data['camera_matrix']['data'], dtype=np.float32).reshape((3, 3))
        self.dist_coeffs = np.array(calib_data['distortion_coefficients']['data'], dtype=np.float32)
        self.rectification_matrix = np.array(calib_data['rectification_matrix']['data'], dtype=np.float32).reshape((3, 3))
        self.projection_matrix = np.array(calib_data['projection_matrix']['data'], dtype=np.float32).reshape((3, 4))
        # Load transforms if available
        if 'transforms' in calib_data:
            for name, transform in calib_data['transforms'].items():
                R = np.array(transform['R'], dtype=np.float32).reshape((3, 3))
                t = np.array(transform['t'], dtype=np.float32).reshape((3,))
                self.transforms[name] = (R, t)
    
    def save_params(self):
        """
        Save the camera parameters to a YAML file, with all `data:` arrays in
        horizontal (flow) style.
        """
        class FlowList(list):
            """A list that PyYAML will emit in [a, b, c] (flow) style."""
            pass

        def _represent_flow_list(dumper, data):
            return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

        yaml.add_representer(FlowList, _represent_flow_list)

        yaml_file = os.path.join(CALIBRATION_DIR, f"{self.name}.yaml")
        calib_data = {
            'image_width': self.width,
            'image_height': self.height,
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': FlowList(self.camera_matrix.ravel().tolist())
            },
            'distortion_coefficients': {
                'rows': 1, 'cols': 5,
                'data': FlowList(self.dist_coeffs.ravel().tolist())
            },
            'rectification_matrix': {
                'rows': 3, 'cols': 3,
                'data': FlowList(self.rectification_matrix.ravel().tolist())
            },
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': FlowList(self.projection_matrix.ravel().tolist())
            },
            'transforms': {
                name: {
                    'R': FlowList(transform[0].ravel().tolist()),
                    't': FlowList(transform[1].ravel().tolist())
                } for name, transform in self.transforms.items()
            }
        }

        os.makedirs(CALIBRATION_DIR, exist_ok=True)
        with open(yaml_file, 'w') as f:
            yaml.dump(calib_data, f, sort_keys=False)

class MultiCamCapture:
    def __init__(self, base_dir: str, cameras: List[Camera], id: str):
        """
        Store the paths to the images for each camera used in a single capture.
        """
        self.base_dir = base_dir
        self.cameras = cameras
        self.id = id
        self.image_paths: Dict[str, str] = {
            cam.name: os.path.join(self.base_dir, cam.name, id + EXTENSION) for cam in cameras
        }
        self.images: Dict[str, np.ndarray] = {}
        
    def load_images(self) -> Dict[str, np.ndarray]:
        """
        Load the images from the image paths.
        """
        for cam_name, path in self.image_paths.items():
            self.images[cam_name] = cv2.imread(path)
        return self.images
    
    def unload_images(self):
        """
        Unload the images to free up memory.
        """
        self.images = {}
    
    def get_image(self, camera_name: str) -> np.ndarray:
        """
        Get the image for a given camera.
        """
        if camera_name not in self.images:
            raise ValueError(f"Camera {camera_name} not found in MultiCamCapture.")
        if self.images[camera_name] is None:
            raise ValueError(f"Image for camera {camera_name} is not loaded.")
        return self.images[camera_name]

def load_multicam_captures(base_dir: str, cameras: List[Camera]) -> List[MultiCamCapture]:
    """
    Load multiple camera captures from a base directory.
    """

    # Find the file ids from the first cameras directory
    file_ids = [
        os.path.splitext(filename)[0]
        for filename in os.listdir(os.path.join(base_dir, cameras[0].name))
        if filename.endswith(EXTENSION)
    ]

    # Create the list of MultiCamCapture objects
    captures = []
    for file_id in file_ids:
        captures.append(MultiCamCapture(base_dir, cameras, file_id))

    return captures

class MultiCamCalibration:
    def __init__(self, cameras: List[Camera], charuco_board: cv2.aruco.CharucoBoard):
        """
        Calibrate a multi-camera system using ChAruCo boards.
        """
        self.cameras = cameras
        self.charuco_board = charuco_board

        charuco_params = cv2.aruco.CharucoParameters()
        charuco_params.tryRefineMarkers = True
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.adaptiveThreshConstant = 9
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        refine_params = cv2.aruco.RefineParameters()
        
        self.charuco_detector = cv2.aruco.CharucoDetector(
            board=charuco_board,
            charucoParams=charuco_params,
            detectorParams=detector_params,
            refineParams=refine_params
        )

        self.min_corners = 8

        self.captures: List[MultiCamCapture] = []

        # 2D image positions of detected ChArUco (chessboard) corners and marker corners
        self.valid_capture_mask: Dict[str, List[bool]] = {cam.name: [] for cam in cameras}
        self.all_charuco_corners: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}
        self.all_charuco_ids: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}
        self.all_marker_corners: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}
        self.all_marker_ids: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}

        # r_vecs and t_vecs are the rotation and translation vectors for each image
        self.valid_r_vecs: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}
        self.valid_t_vecs: Dict[str, List[np.ndarray]] = {cam.name: [] for cam in cameras}

    def add_capture_dir(self, data_dir: str):
        self.captures = load_multicam_captures(data_dir, self.cameras)
        if self.captures is None or len(self.captures) == 0:
            raise ValueError(f"No captures found in directory: {data_dir}")

    def detect_corners(self):
        """
        Detect the corners of the ChAruCo board for each camera in the multi-camera system.
        """
        if self.captures is None or len(self.captures) == 0:
            raise ValueError("No captures have been added to the calibration object.")

        # Loop through each capture set and detect the board and corners
        for i, capture in enumerate(self.captures):

            # Load the images for each camera
            capture.load_images()

            # Detect the board and corners for each camera
            for cam in self.cameras:
                img = capture.get_image(cam.name) # Just using the first image if there are multiple

                # Convert to grayscale if necessary
                if len(img.shape) == 3:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                # Set the camera width and height if they haven't been set
                if cam.width == 0 or cam.height == 0:
                    cam.height, cam.width = gray.shape[:2]

                # Get the corners and ids for the ChAruCo board and the marker corners and ids
                """
                charuco_corners   -> ndarray (N×1×2 float)   : 2D image positions of detected ChArUco (chessboard) corners  
                charuco_ids       -> ndarray (N×1 int)       : Unique IDs for each detected ChArUco corner  
                marker_corners    -> list of length M of 4×2 float arrays : Pixel coordinates of all detected ArUco marker corners  
                marker_ids        -> ndarray (M×1 int)       : IDs of detected ArUco markers (in same order as markerCorners)"
                """
                charuco_corners, charuco_ids, marker_corners, marker_ids = self.charuco_detector.detectBoard(gray)

                # Only store detections if enough corners were found
                if charuco_ids is not None and len(charuco_ids) >= self.min_corners:
                    self.valid_capture_mask[cam.name].append(True)
                    self.all_charuco_corners[cam.name].append(charuco_corners)
                    self.all_charuco_ids[cam.name].append(charuco_ids)
                    self.all_marker_corners[cam.name].append(marker_corners)
                    self.all_marker_ids[cam.name].append(marker_ids)
                else:
                    self.valid_capture_mask[cam.name].append(False)
                    self.all_charuco_corners[cam.name].append(np.array([]))
                    self.all_charuco_ids[cam.name].append(np.array([]))
                    self.all_marker_corners[cam.name].append(np.array([]))
                    self.all_marker_ids[cam.name].append(np.array([]))

            # Unload the images to save memory
            capture.unload_images()
            print(f"Capture {i+1}/{len(self.captures)}")

    def compute_intrinsics(self, camera: Camera):
        """
        Compute the intrinsics for a single camera.
        """
        if len(self.all_charuco_corners[camera.name]) == 0:
            print(f"Warning: No detections for camera {camera.name}. Skipping calibration.")
            return
        
        # Calibrate the camera (filter out empty detections)
        mask = self.valid_capture_mask[camera.name]
        valid_corners = [c for c, valid in zip(self.all_charuco_corners[camera.name], mask) if valid]
        valid_ids = [i for i, valid in zip(self.all_charuco_ids[camera.name], mask) if valid]
        err, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=valid_corners,
            charucoIds=valid_ids,
            board=self.charuco_board,
            imageSize=(camera.width, camera.height),
            cameraMatrix=None,
            distCoeffs=None
        )

        # Save the camera data
        camera.error = err
        camera.camera_matrix = camera_matrix
        camera.dist_coeffs = dist_coeffs

        # Maintain indexing from the original list
        rvecs, tvecs = list(rvecs), list(tvecs)
        self.valid_r_vecs[camera.name] = [rvecs.pop(0) if mask[i] else np.array([]) for i in range(len(mask))]
        self.valid_t_vecs[camera.name] = [tvecs.pop(0) if mask[i] else np.array([]) for i in range(len(mask))]

        # Set the projection matrix to the camera matrix for now, will be updated later if stereo camera
        camera.projection_matrix[:3, :3] = camera_matrix 

        return err, camera_matrix, dist_coeffs, rvecs, tvecs

    def compute_stereo_rectification(self, camera0: Camera, camera1: Camera):
        """
        Compute the stereo rectification parameters between two cameras. This 
        uses the Levenberg-Marquardt algorithm to minimize the reprojection error.
        """
        # Check for detections in both cameras
        if len(self.all_charuco_corners[camera0.name]) == 0 or len(self.all_charuco_corners[camera1.name]) == 0:
            print(f"Warning: No detections for cameras {camera0.name} and {camera1.name}. Skipping calibration.")
            return
        
        # Match the object points and image points for the two cameras across all captures
        capture_obj_pts = []
        capture_img_pts0 = []
        capture_img_pts1 = []
        for cam0_pts, cam0_ids, cam1_pts, cam1_ids in zip(self.all_charuco_corners[camera0.name], self.all_charuco_ids[camera0.name],
                                                          self.all_charuco_corners[camera1.name], self.all_charuco_ids[camera1.name]):
            
            # Skip empty/invalid detections
            if len(cam0_pts) < self.min_corners or len(cam1_pts) < self.min_corners:
                continue

            # Match the object points (3D coordinates relative to the ChAruCo board) and image points (2D pixel coordinates)
            obj_pts0, img_pts0 = self.charuco_board.matchImagePoints(detectedCorners=cam0_pts, detectedIds=cam0_ids)
            obj_pts1, img_pts1 = self.charuco_board.matchImagePoints(detectedCorners=cam1_pts, detectedIds=cam1_ids)

            # Find the common object points and image points
            _, common_idx0, common_idx1 = np.intersect1d(cam0_ids.flatten(), cam1_ids.flatten(), return_indices=True)
            aligned_obj = obj_pts0[common_idx0]
            aligned_img0 = img_pts0[common_idx0]
            aligned_img1 = img_pts1[common_idx1]

            # Store the object points from this capture
            capture_obj_pts.append(aligned_obj)
            capture_img_pts0.append(aligned_img0)
            capture_img_pts1.append(aligned_img1)

        # Compute the stereo rectification parameters
        ret, cm1, dc1, cm2, dc2, R, T, E, F = cv2.stereoCalibrate(
            objectPoints=capture_obj_pts,
            imagePoints1=capture_img_pts0,
            imagePoints2=capture_img_pts1,
            cameraMatrix1=camera0.camera_matrix,
            distCoeffs1=camera0.dist_coeffs,
            cameraMatrix2=camera1.camera_matrix,
            distCoeffs2=camera1.dist_coeffs,
            imageSize=(camera0.width, camera0.height),
            # criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-5),
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        # Stereo‑rectify → get R1, R2, P1, P2, Q
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            cameraMatrix1=cm1, distCoeffs1=dc1,
            cameraMatrix2=cm2, distCoeffs2=dc2,
            imageSize=(camera0.width, camera0.height),
            R=R, T=T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0
        )

        # Store results
        camera0.rectification_matrix = R1
        camera0.projection_matrix    = P1
        camera1.rectification_matrix = R2
        camera1.projection_matrix    = P2

        return ret, R1, R2, P1, P2, Q

    def compute_extrinsics(self, camera0: Camera, camera1: Camera):
        """
        Compute the extrinsics between two cameras using camera 0 as the reference.
        This function assumes that the cameras are already intrinsically calibrated.
        """

        # Initial guess: identity rotation, zero translation
        x0 = np.zeros(6, dtype=np.float64)

        def residuals(x):
            R = cv2.Rodrigues(x[0:3])[0]
            t = x[3:6].reshape(3, 1)
            residual_list = []
            for r_vec0, t_vec0, r_vec1, t_vec1 in zip(
                    self.valid_r_vecs[camera0.name], 
                    self.valid_t_vecs[camera0.name], 
                    self.valid_r_vecs[camera1.name], 
                    self.valid_t_vecs[camera1.name]):
                
                # Skip empty/invalid detections
                if r_vec0.size == 0 or r_vec1.size == 0:
                    continue
                
                R0 = cv2.Rodrigues(r_vec0)[0]
                R1 = cv2.Rodrigues(r_vec1)[0]

                R1_pred = R @ R0
                t1_pred = R @ t_vec0.reshape(3,1) + t

                # Compute the difference in axis angle representation
                R_delta = R1_pred.T @ R1
                r_resid = cv2.Rodrigues(R_delta)[0].ravel()

                # Compute the difference in translation
                t_resid = (t_vec1 - t1_pred).ravel()

                residual_list.append(r_resid)
                residual_list.append(t_resid)
            
            return np.hstack(residual_list)

        # Run least squares
        res = least_squares(residuals, x0, method='lm')

        # Extract the optimized parameters
        R = cv2.Rodrigues(res.x[0:3])[0]
        t = res.x[3:6].reshape(3, 1)

        # Save the extrinsics
        camera0.transforms[camera1.name] = (R, t)
        camera1.transforms[camera0.name] = (R.T, -R.T @ t)

        return res.cost, R, t

    # def compute_extrinsics(self, cam0: Camera, cam1: Camera,
    #                     method=cv2.CALIB_HAND_EYE_TSAI):
    #     # Gather valid board poses
    #     R_gripper2base, t_gripper2base = [], []   #  == cam0←board
    #     R_target2cam , t_target2cam  = [], []     #  == board←cam1

    #     for r0, t0, r1, t1 in zip(self.valid_r_vecs[cam0.name],
    #                             self.valid_t_vecs[cam0.name],
    #                             self.valid_r_vecs[cam1.name],
    #                             self.valid_t_vecs[cam1.name]):

    #         if r0.size == 0 or r1.size == 0:
    #             continue

    #         # board → cam  (what Charuco gives us)
    #         R0, t0 = cv2.Rodrigues(r0)[0], t0.reshape(3,1)
    #         R1, t1 = cv2.Rodrigues(r1)[0], t1.reshape(3,1)

    #         # ---- build A_i  :  cam0 → board   (invert R0,t0) ----
    #         R_cam0_board = R0.T
    #         t_cam0_board = -R0.T @ t0

    #         # ---- build B_i  :  board → cam1  (just R1,t1) ----
    #         R_board_cam1 = R1
    #         t_board_cam1 = t1

    #         R_gripper2base.append(R_cam0_board)
    #         t_gripper2base.append(t_cam0_board)
    #         R_target2cam.append(R_board_cam1)
    #         t_target2cam.append(t_board_cam1)

    #     if len(R_gripper2base) < 3:
    #         raise RuntimeError("Need ≥3 common captures for hand-eye")

    #     R, t = cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
    #                                 R_target2cam , t_target2cam ,
    #                                 method=method)   # TSAI, PARK, etc.

    #     cam0.transforms[cam1.name] = (R, t)
    #     cam1.transforms[cam0.name] = (R.T, -R.T @ t)   # store inverse

    #     return R, t

def main():

    # # Retrieve folder containing the capture sets from input arguments
    # if len(sys.argv) < 2:
    #     print("Usage: python multi_cam_calibration.py <data_folder>")
    #     sys.exit(1)

    # data_dir = sys.argv[1]

    # # Expand the data folder path if it contains '~'
    # data_dir = os.path.expanduser(data_dir)
    data_dir = "/home/hayden/cmu/kantor_lab/ros2_ws/calibration_images"
    # data_dir = "/home/hayden/cmu/kantor_lab/calibration/charuco_board3_3-28/images"

    # Construct the camera objects
    firefly_left = Camera("firefly_left")
    firefly_right = Camera("firefly_right")
    ximea = Camera("ximea")
    zed_left = Camera("zed_left")
    zed_right = Camera("zed_right")
    cameras = [firefly_left, firefly_right, ximea, zed_left, zed_right]

    # Define the ChAruCo board parameters
    ARUCO_DICT = cv2.aruco.DICT_5X5_50
    SQUARES_VERTICALLY = 6
    SQUARES_HORIZONTALLY = 4
    SQUARE_LENGTH = 0.04
    MARKER_LENGTH = 0.03

    # Create the ChAruCo board
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    charuco_board = cv2.aruco.CharucoBoard(
        size = (SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), 
        squareLength = SQUARE_LENGTH, 
        markerLength = MARKER_LENGTH, 
        dictionary = dictionary
    )

    # Create the multi-camera calibration object
    multi_cam_calib = MultiCamCalibration(cameras, charuco_board)
    multi_cam_calib.add_capture_dir(data_dir)

    # Calibrate the cameras
    multi_cam_calib.detect_corners()

    # Compute the intrinsics for each camera
    for cam in cameras:
        err = multi_cam_calib.compute_intrinsics(cam)[0]
        print(f"Camera {cam.name} intrinsics computed with error {err:.4f}.")
        cam.save_params()

    # Compute the rectification parameters for the stereo pairs
    err = multi_cam_calib.compute_stereo_rectification(firefly_left, firefly_right)[0]
    print(f"Stereo rectification between firefly_left and firefly_right computed with error {err:.4f}.")
    err = multi_cam_calib.compute_stereo_rectification(zed_left, zed_right)[0]
    print(f"Stereo rectification between zed_left and zed_right computed with error {err:.4f}.")

    # Compute the extrinsics between the firefly_left and all other cameras
    multi_cam_calib.compute_extrinsics(firefly_left, firefly_right)
    print(f"Extrinsics between firefly_left and firefly_right computed with error {err:.4f}.")
    multi_cam_calib.compute_extrinsics(firefly_left, ximea)
    print(f"Extrinsics between firefly_left and ximea computed with error {err:.4f}.")
    multi_cam_calib.compute_extrinsics(firefly_left, zed_left)
    print(f"Extrinsics between firefly_left and zed_left computed with error {err:.4f}.")
    multi_cam_calib.compute_extrinsics(firefly_left, zed_right)
    print(f"Extrinsics between firefly_left and zed_right computed with error {err:.4f}.")
    firefly_left.save_params() # Save as we go

    # Save the camera parameters for all cameras
    for cam in cameras:
        cam.save_params()
        print(f"Camera {cam.name} parameters saved.")

if __name__ == "__main__":
    main()
