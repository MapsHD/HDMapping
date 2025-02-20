import Release.lidar_odometry_py as lo
import Release.multi_view_tls_registration_py as mvr
from pathlib import Path

DATA_DIR = Path(
    R"C:\Users\Admin\Documents\Mosaic\hdmapping_experiments\lidar_1_automatic"
)

lo_params = lo.LidarOdometryParams()
lo_params.in_out_params.resolution_X = 0.3
lo_params.in_out_params.resolution_Y = 0.3
lo_params.in_out_params.resolution_Z = 0.3
lo_params.save_laz = True
lo_params.save_trajectory = True
lo.run_lidar_odometry(str(DATA_DIR), lo_params)

mvr_params = mvr.TLSRegistration()
mvr_params.initial_pose_to_identity = True
mvr_params.use_icp = True
mvr.run_multi_view_tls_registration(
    str(DATA_DIR / R"lidar_odometry_result_6\session.json"),
    mvr_params,
    lo_params.current_output_dir,
)
