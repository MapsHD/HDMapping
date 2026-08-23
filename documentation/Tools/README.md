# Camera-LiDAR calibration tools

Three tools (`apps/camera_lidar_*`) supports camera calibration with LIDAR and perofming applying colors to data from session.

- **camera_lidar_intrinsics_calib** -- checkerboard-based camera intrinsic calibration (OpenCV rational distortion model).
- **camera_lidar_calibration** -- interactive LiDAR-camera extrinsic calibration: aligns a LAZ/LAS point cloud to a camera image with live GPU-shader reprojection feedback.
- **camera_lidar_trajectory_viewer** -- multi-camera trajectory/point-cloud viewer that assigns per-point "which camera colored this point" RGB, with LAZ export, plus optional COLMAP sparse-model and ROS 2 bag export.

Watch video [[calib]](https://www.youtube.com/watch?v=2xjZ3sbX8oA)

# GNSS with RTK

A portable NTRIP (Networked Transport of RTCM via Internet Protocol) client for M5Stack devices that receives RTK correction data from NTRIP casters and forwards it to u-blox GNSS receivers for high-precision positioning https://github.com/michalpelka/M5NtripClient.
