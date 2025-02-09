import pyzed.sl as sl

# 1. Initialize camera with positional tracking
init_params = sl.InitParameters()  
init_params.camera_resolution = sl.RESOLUTION.HD720  
init_params.coordinate_units = sl.UNIT.METER  
init_params.depth_minimum_distance = 0.2  
camera = sl.Camera()
if camera.open(init_params) != sl.ERROR_CODE.SUCCESS:
    exit(1)

tracking_params = sl.PositionalTrackingParameters()  
camera.enable_positional_tracking(tracking_params)

# 2. Prepare runtime parameters for capturing
runtime_params = sl.RuntimeParameters()
spatial_mapping_params = sl.SpatialMappingParameters()
camera.enable_spatial_mapping(spatial_mapping_params)

# 3. Grab frames & IMU, update 3D scene
mesh = sl.Mesh()
while True:
    if camera.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve IMU data (sensor_data)
        sensor_data = sl.SensorsData()
        camera.get_sensors_data(sensor_data, sl.TIME_REFERENCE.CURRENT)
        # Retrieve pose from positional tracking
        pose = sl.Pose()
        tracking_state = camera.get_position(pose, sl.REFERENCE_FRAME.WORLD)
        # Update spatial mapping
        camera.request_spatial_map_async()

        # ...existing code...
        if camera.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS:
            camera.retrieve_spatial_map_async(mesh)

        # ...e.g. break on key press or after enough frames...
        # (Pseudo-code) if should_stop(): break

# 4. Save 3D mesh
mesh.save("scene_mesh.obj")
camera.disable_spatial_mapping()
camera.disable_positional_tracking()
camera.close()