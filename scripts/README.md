# Repository management utilities

* `run_clang_format.py` — runs `clang-format` on all C/C++ source and header files located in:

  * `core`
  * `apps`
  * `pybind`
  * `shared`
  * `raylib_widgets`

* `run_check_clang_format.py` — checks whether running `clang-format` would modify any tracked project files. The script runs `run_clang_format.py` and then checks the Git working tree for changes. If formatting would produce changes, the check fails and lists the affected files.

* `run_submodules_sync_update.py` — initializes and updates Git submodules recursively using:
  `git submodule sync` and `git submodule update --init --recursive`.

* `run_pull_submodules_changes.py` — updates submodules located under `3rdparty`. For each submodule, the script:

  * reads the branch configured in `.gitmodules`;
  * initializes the submodule if it is missing;
  * fetches from `origin`;
  * switches to the configured branch;
  * performs a fast-forward-only pull;
  * reports whether the submodule was actually updated.

  Submodules without a branch configured in `.gitmodules` are reported and skipped.

# Data utilities

* `resample_imu.py` — resamples Mandeye IMU CSV files to a lower rate (default 200 Hz) by averaging all samples in each 1/rate window, so vibration above the new Nyquist rate is attenuated instead of aliased. Handles `imu****.csv` and external-IMU subfolders such as `MURATA_IMU/`. Given a scan folder, it writes a copy (`<folder>_imu200hz` by default; other files hard-linked when possible) with the IMU resampled, leaving the input untouched:

  ```
  python3 scripts/resample_imu.py /path/to/continousScanning_0013 --rate 200
  ```

* `lio_to_tum.py` — converts a `lidar_odometry_step_1` result folder (`lio_result_N`) to a TUM trajectory (`timestamp tx ty tz qx qy qz qw`). Uses `session_ini_poses.mri`, the poses step 1 wrote, because steps 2 and 3 overwrite `session_poses.mrp`. Standard library only:

  ```
  python3 scripts/lio_to_tum.py /path/to/lio_result_0 trajectory.tum
  ```

* `lio_ate.py` — absolute trajectory error of a TUM trajectory against a golden one: poses paired in order with timestamps within `--max-dt-s` (default 10 µs), position RMSE without alignment. Exits 0 when the RMSE is within `--max-ate-m` (default 0.001 m), 1 when it is not, 2 when the poses or timestamps differ. Standard library only:

  ```
  python3 scripts/lio_ate.py golden.tum trajectory.tum --max-ate-m 0.001
  ```

* `plot_session_uncertainty.py` — plots the CSV written by `session_uncertainty_calculation` (any `--method`): a top view of the trajectory coloured by the position sigma with the x-y covariance ellipses, and the pose sigmas over time with sigma0 and correspondences (or converged Monte Carlo runs). Shows the figures, or saves them with `--output`. Needs `numpy` and `matplotlib`:

  ```
  python3 scripts/plot_session_uncertainty.py uncertainty.csv --output uncertainty.png
  ```
