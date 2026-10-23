# Easy GUI: design notes

This document explains how Easy GUI works, why it is built the way it is, and how its output was validated. It is for
collaborators and reviewers who know HDMapping. The user guide is [../README.md](../README.md).

Naming: the user-facing name is **Easy Forest HDMapping** (window title, user guide, release packages). Internally,
the code keeps the name Easy GUI (`apps/easy_gui`, the `easy_gui` binary and targets), and so does this document.

Version: Easy Forest HDMapping has its own version (`EASY_FOREST_VERSION` in `apps/easy_gui/CMakeLists.txt`,
equal to the `easy-forest-v*` tag). Window title, README.txt and easy_gui.log show both, e.g. "Easy Forest HDMapping
0.2.0 (HDMapping 0.105.0)". The LAZ header's 32-character fields cannot hold that: generating software is
"Easy Forest HDMapping 0.2.0", system identifier "HDMapping 0.105.0".

Contents:

1. [Goal and scope](#1-goal-and-scope)
2. [Architecture](#2-architecture)
3. [Background: Mandeye raw data](#3-background-mandeye-raw-data)
4. [Background: what lidar odometry (step 1) does to points](#4-background-what-lidar-odometry-step-1-does-to-points)
5. [Design decisions](#5-design-decisions)
6. [The export](#6-the-export)
7. [Validation](#7-validation)
8. [Experimental option: place every point by its raw time](#8-experimental-option-place-every-point-by-its-raw-time)
9. [Resources: time, memory, disk](#9-resources-time-memory-disk)
10. [Batch mode](#10-batch-mode)
11. [Status and next steps](#11-status-and-next-steps)

---

## 1. Goal and scope

One window that takes a raw Mandeye recording folder, runs HDMapping's lidar odometry (step 1) with the precision
forestry preset, and exports two files:

- a LAZ point cloud with every per-point attribute that is available, and
- a trajectory CSV with timestamp, position, orientation and every other available field.

Constraints:

- Built on the raylib + rlImGui GUI stack (like steps 2 and 3).
- Reuse HDMapping's code; do not duplicate its algorithms. `core_math` is not modified, and the existing apps behave
  as before.
- Show progress and report errors in plain language.

Since 0.2.0 it also processes several recordings in one go (batch mode, §10), each into its own output folder.

Not in scope (v1): step 2 refinement, step 3 (multi-session), georeferencing (GNSS, control points), and rigs with
more than one lidar. The pre-flight check refuses recordings with more than one lidar id.

Planned for v2: pose-graph loop closure behind a toggle, using `PoseGraphLoopClosure` with automatically detected
candidates, where an edge is rejected if its ICP fails. `PoseGraphSLAM::optimize` will not be used.

## 2. Architecture

```
recording folder (read only)
   │
   ├─ pre-flight check ────────── file pairing, LAZ/IMU readability, lidar ids, calibration, clocks, disk space
   │
   ├─ staging ─────────────────── output/work/staging/: symlinks to the recording's files
   │
   ├─ lidar_odometry_step_1 ───── child process: <staging> <params.toml> <output/work/lio_session>
   │      (forestry preset)        progress parsed from its log
   │
   └─ export ──────────────────── point_cloud.laz, trajectory.csv, README.txt
```

Code in `apps/easy_gui`:

| Part | Role |
|---|---|
| `easy_gui_core` (static library, no raylib) | Everything except the window, so the doctest suite can link it. |
| `src/preflight.*` | Classifies files exactly like step 1's `load_data` does and checks them (see §3). |
| `src/staging.*` | Builds the staging folder of symlinks (see §5.6). |
| `src/child_process.*` | Runs step 1: POSIX `posix_spawn` with its own process group; Windows `CreateProcess` with a job object (built and tested on the Windows CI runner). |
| `src/progress_parser.*` | Turns step 1's log lines into phases and a progress fraction. |
| `src/forestry_preset.*` | Writes the step 1 parameter file with step 1's own `TomlIO`. |
| `src/export.*`, `src/las14_writer.*` | The export (§6) and a streaming LAS 1.4 point format 6 LAZ writer with a hand-written extra-bytes VLR. |
| `src/pipeline.*` | Worker thread that runs the stages above and publishes status for the GUI. |
| `src/batch.*` | Batch mode (§10): discovery, output names, done markers, batch disk check, and `BatchRunner`, which runs `Pipeline` once per recording. |
| `src/sleep_blocker.*` | Keeps the computer awake while a batch runs (§10.5). |
| `src/easy_gui_main.cpp` | The window, plus the command-line modes below. |
| `tests/` | `easy_gui_tests` (doctest). |
| `scripts/check_export.py` | Acceptance checks for an export (§7). |

`easy_gui_core` compiles two step 1 sources by path, like the other apps do: `lidar_odometry_utils.cpp` (for the
calibration parser the pre-flight check uses) and `toml_io.cpp` (to write the parameter file and to read it back from
the session). It is built with `WITH_GUI=0` and links `core_no_gui` (for `Session`) and `core_math`, not `core`.
Because of that, `core_no_gui` is now always built; before, it was only built with tests, pybind or the CLI tools.

**Why step 1 runs as a child process.** Crash isolation (step 1 calls `std::exit` on some input errors and opens a
native message box on `bad_alloc`), and a real cancel: killing the process group stops it immediately and frees its
~8.5 GB. Building `easy_gui` also builds `lidar_odometry_step_1` (`add_dependencies`). Easy GUI finds it next to its own
executable, under the plain name or the `HDMAPPING_BINARY_PREFIX` name; `HDMAPPING_STEP1_EXECUTABLE` overrides the
lookup.

**Building only Easy GUI:** `cmake -B build -DHDMAPPING_EASY_GUI_ONLY=ON` builds and installs only `easy_gui`,
`lidar_odometry_step_1` and the libraries they link (plus `easy_gui_tests` with `BUILD_TESTING=ON`). The default is
off, which leaves the upstream build unchanged. On macOS this mode also sets `CMAKE_OSX_DEPLOYMENT_TARGET` to 13.0
(unless given), the lowest version the code builds for; without it the minimum is the build machine's macOS version.

Quirks of headless step 1 (`lidar_odometry_step_1 <input> <params.toml> <output>`) that Easy GUI handles:

- The exit code is 0 even after a caught exception, so success is judged by the presence of `session.mjs`.
- `load_data` waits on stdin before `std::exit` when the calibration does not match, so the child gets a closed stdin.
  The pre-flight check repeats `load_data`'s calibration checks so that this case is refused before step 1 starts.
- Progress exists only as log lines: `running iterations: i/n` during setup and `finished optimizing … - i / N …` per
  odometry chunk.

**Command-line modes** of `easy_gui`:

| Mode | Purpose |
|---|---|
| `--headless <recording> <output>` | The whole pipeline without a window. Exit code 0 success, 1 failed or refused, 2 usage. |
| `--batch <parent folder> <output>` | Batch mode without a window (§10). Exit code 0 when every selected recording succeeded or was already done, 1 otherwise or refused, 2 usage. Ctrl+C once = stop after current, twice = stop now. |
| `--export-only <recording> <session.mjs> <output>` | Only the export, on an existing step 1 session (about 2.5 min instead of about 22). |
| `--check-band <m>` (with any of the above) | Test mode for the alignment check, see §7.3. |
| `--place-by-raw-time` (with any of the above) | The experimental export option of §8. |
| `--screenshot <file.png> [recording \| --batch <parent folder>]` | Renders the window for about 2 s, saves it and exits (for documentation). raylib saves into the working directory. `EASY_GUI_SCREENSHOT_START=1` starts the batch once it is ready and `EASY_GUI_SCREENSHOT_FRAME=<n>` takes the picture at frame n (30 per second), to capture a running or finished batch. |

**Output folder:**

| Path | Content |
|---|---|
| `point_cloud.laz` | The point cloud (§6). |
| `trajectory.csv` | The trajectory (§6.4). |
| `README.txt` | Field descriptions, time base and near-point rule, generated with the run's own values. |
| `easy_gui.log` | Everything, including step 1's full output. |
| `work/easy_gui_step1.toml` | The parameters step 1 ran with. |
| `work/lio_session/` | The step 1 session (`session.mjs` etc.). Kept, so it can be opened in step 2 for refinement or georeferencing. |
| `work/staging/` | Removed after success unless "Keep intermediate files" is ticked. |
| `easy_gui_done.txt` | Batch mode only: written once the result is complete (§10.4). |

After a failure or a cancel, `work/` is removed, except that an export failure keeps `work/lio_session/`, since
odometry took most of the run.

## 3. Background: Mandeye raw data

Verified on a real recording and in the Mandeye firmware source.

**LAZ files** (`lidar*.laz`): LAS 1.2, point format 1.

- Real fields: X, Y, Z in the sensor frame, intensity (0–160 in practice), gps_time, and user_data = lidar id. There
  are no extra bytes; every other standard field is 0.
- gps_time is Unix epoch seconds. The header wrongly flags it as adjusted GPS time.
- classification is a constant 20 and has no meaning: the firmware never copies the Livox point tag. A planned firmware
  fix will store the tag there (bits 0–1 adhesion, 2–3 particles, 4–5 other).
- About 15 % of the records are "no return" points: exactly (0, 0, 0) or gps_time 0.

**IMU files** (`imu*.csv`, 200 Hz): timestamp (ns, the same clock as the LAZ gps_time), gyro XYZ, acc XYZ (g), imuId,
and timestampUnix (ms, the host computer's clock, not synchronised with the others).

**How step 1's `load_data` picks files** (the pre-flight check mirrors this):

- It sorts all files, then pairs `lidar*.laz` with `imu*.csv` **by position**, and loads `min(#laz, #csv)` pairs. A
  mismatch of the 4-digit index only prints a warning; surplus trailing files are skipped.
- `status*` and `cam*` JSON files are ignored. Any other `.json`, or a `.mjc`, is taken as the calibration file. Only
  the first `.sn` file is used.
- It writes `cache/` and `preview/` into the folder that holds the input files.

## 4. Background: what lidar odometry (step 1) does to points

Verified in `compute_step_1` and `save_result` (`apps/lidar_odometry_step_1`). These facts drive most decisions in §5.

**Timestamps are not preserved.** Step 1 works in worker chunks of 20 trajectory poses (≈ 0.1 s). In each chunk it
overwrites every point's timestamp with a uniform ramp `t0 + k·(t_last − t0)/N` ("correct points timestamps" in
`lidar_odometry.cpp`). Raw gps_time therefore cannot be joined to step 1's output points, with or without a tolerance.

Output formats: gps_time in `scan_lio_N.laz` and the timestamps in `trajectory_lio_N.csv` are seconds × 1e9 (ns stored
as a double, ≈ 256 ns resolution). `scan_lio_N.laz` holds only XYZ (in the frame of the scan's first trajectory node),
intensity and gps_time; the lidar id is dropped in `points_to_vector`.

**Distance filters:**

- `filter_threshold_xy_inner` / `_outer` act at load time (`load_point_cloud`), so they remove points from the odometry
  input *and* from the output. They measure the **horizontal** radius √(x² + y²) in the raw sensor frame.
- `threshould_output_filter` is **output only** (`points_to_vector` in `save_result`, `fill_session_from_worker_data`;
  the optimisers never read it) and measures the **3D** norm in the sensor frame.
- Consequently an output filter below `filter_threshold_xy_inner` has no effect: every surviving point already has a 3D
  norm ≥ its horizontal radius > inner.

**Other points step 1 drops:** gps_time 0; points before the first `threshold_initial_points`; the gap
`[t_last, t_next_first)` between consecutive worker chunks (≈ 1 of every 20 IMU intervals, about 5 % of the time); the
first point of each chunk; chunks with ≤ 1000 points; the trailing poses that do not fill a whole chunk; and the last
concatenated group if it has ≤ `min_counter_concatenated_trajectory_nodes` nodes.

Voxel decimation (`params.decimation`) only thins the points used for optimisation; `scan_lio_N.laz` is written from
the undecimated originals.

**Per-point range is recoverable without the raw files:** `index_pose` = the last trajectory node with t_node < t_point
(step 1's own rule, applied to the ramp time), then `p_sensor = csv_pose[index_pose]⁻¹ · p_laz`, both relative to the
scan's first node. (`save_index_pose` would record the index directly, but it is not in `TomlIO`'s attribute list, so a
parameter file cannot enable it.)

## 5. Design decisions

### 5.1 Output format: LAS 1.4, point format 6, extra bytes

Point format 6 has a 64-bit gps_time and room for extra bytes. Two extra-bytes attributes are written:

- `range_m` (float32): 3D distance from the lidar at the moment of measurement.
- `point_origin` (uint8): where the point comes from (0 = step 1, 1 = near point, 2 = gap point, 3 = test mode only).

The extra-bytes VLR is written by hand because `laszip_add_attribute` sets the scale bit, which makes readers such as
laspy decode the fields as float64. With the hand-written VLR, CloudCompare 2.x loads both as named scalar fields and
laspy reads them as float32 / uint8.

`user_data` keeps the lidar id; `point_source_id` is the step 1 scan index (the `scan_lio_N.laz` number, about 5 s of
recording).

### 5.2 Time base

`gps_time` is Unix epoch seconds, not GPS week time and not adjusted standard GPS time, so global encoding bit 0 (GPS
time type) is 0. README.txt states this. For the default export, step 1's own points carry step 1's ramp time (§4),
typically 1–4 ms and at most about 20 ms from the real measurement time; near and gap points carry their raw time.

(Step 2's own `--export` writes gps_time × 1e18, because it multiplies the session's ns-valued times by 1e9 again.
Easy GUI does not reproduce that.)

### 5.3 The default export is "hybrid"

The point cloud combines:

- `point_origin = 0`: step 1's own points, placed exactly as step 1 placed them;
- `point_origin = 1`: **near points**, raw points step 1's distance filters removed (§5.4);
- `point_origin = 2`: **gap points**, raw points inside step 1's distance limits that step 1 skipped between its 0.1 s
  pieces (§5.5).

Near and gap points come from the raw files and are placed at their real gps_time with step 1's rule: the last
trajectory node with t_node < t, if it is within 10 ms (step 1's own tolerance), so
`global = mrp[scan] · csv_pose[node] · calib · p_raw`.

Placing step 1's own points by raw time as well is possible (§8) but measured slightly less sharp, so it is only an
experimental option.

### 5.4 Near points

Step 1 drops every point whose horizontal range is below 1.0 m, which leaves a hole around the walking path. Easy GUI
adds those points back at export; odometry never sees them.

Rule: keep raw points with `near_min_range` ≤ 3D range **and** that step 1's distance filters removed. As implemented
this is the exact complement of step 1's filters: horizontal range ≤ `filter_threshold_xy_inner` (1.0 m) or 3D norm ≤
`threshould_output_filter`, and horizontal range < `filter_threshold_xy_outer`. No-return points are always dropped.

`near_min_range` defaults to 0.4 m (advanced setting "Closest points kept", 0.1–0.95 m). In measured raw data the
operator's body and the handle show up only as small blobs inside 0.4 m, and near points beyond that are environment in
every direction.

**Output filter.** For the near rule to meet step 1's output without a gap, Easy GUI sets
`threshould_output_filter = filter_threshold_xy_inner` (1.0 m instead of the preset's 1.5 m). Because the output filter
is output only (§4), the trajectory is unchanged; this was confirmed byte for byte (§7.1). Points with horizontal range
≥ 1.0 m then all come from step 1, and points below come from the near rule.

### 5.5 Gap points

Step 1 takes `[first node, last node)` of each 0.1 s piece, so the raw points between a piece's last node and the next
piece's first node are never exported (about 5 % of the recording time). Easy GUI adds them back: a raw point inside
step 1's distance limits is a gap point when the last trajectory node at or before its time is the last node of a
piece, i.e. `j % threshold_nr_poses == threshold_nr_poses − 1` for node j within the scan.

### 5.6 Never write into the recording

The recording may be on the scanner's USB stick, so Easy GUI never writes into it. Because `load_data` writes `cache/`
and `preview/` next to its input files (§3), step 1 runs on a staging folder of symlinks inside the Easy GUI output
folder. The pre-flight check refuses an output folder inside the recording.

### 5.7 Forestry preset

The preset is `LidarOdometryParams` defaults plus the 18 overrides of "Presets → 2 … precise forestry" in
`lidar_odometry_gui.cpp`. HDMapping has no shared preset function (the values are inline GUI code, also copied into the
`precision_forestry` app), so `forestry_preset.cpp` repeats them and writes the TOML through step 1's own `TomlIO`.
The only deviation is the output filter of §5.4.

## 6. The export

Source: `src/export.cpp`.

### 6.1 Flow and memory

1. Load the session with `Session` in `load_cache_mode`: poses from `session_poses.mrp` plus the trajectories, no
   points. This is exactly what step 2 uses.
2. Stream one `scan_lio_N.laz` at a time (step 1's points).
3. Stream one raw LAZ at a time (near and gap points).

The distance thresholds are read from the session's own `HDMapping_params_*.toml`, i.e. what step 1 actually ran with.
Peak memory on the full test recording: 106 MB.

### 6.2 Step 1's points

These replicate step 2's `save_all_to_las`: `m_pose · (p − offset) + offset`, gps_time 0 skipped, scale 0.0001. The
result is integer-identical to `multi_view_tls_registration_step_2 --export` (§7.2).

`range_m` of a step 1 point is `|calib⁻¹ · csv_pose[node]⁻¹ · p_local|`, with the node chosen by step 1's rule on the
ramp time.

### 6.3 Point counts on the test recording

10-minute forest walk, 115 file pairs, 115.9 M raw points of which 17.1 M are no-return records.

| point_origin | Points |
|---|---|
| 0, step 1 | 89,326,756 |
| 1, near | 4,038,530 |
| 2, gap | 4,026,788 |
| total | 97.39 M |

point_cloud.laz 948 MB, trajectory.csv 20 MB (114,940 poses). Export time 143 s.

148 step 1 points (0.0002 %) have a range of 70–99.7 m. These are steep returns (elevation up to 54°): step 1's 70 m
limit is horizontal.

### 6.4 Trajectory CSV

One row per trajectory node: `timestamp_unix_s`, `x, y, z`, `qw, qx, qy, qz`, `om_deg, fi_deg, ka_deg` (HDMapping
Tait-Bryan angles), `scan_index` (= `point_source_id`), `host_clock_unix_s` (the IMU file's host clock, ms resolution,
not synchronised), and `imu_om_deg, imu_fi_deg, imu_ka_deg` (IMU-only orientation before odometry). Timestamps have
1 µs resolution.

## 7. Validation

`scripts/check_export.py <output folder> [--step2 <folder>]` automates the checks below (fields, range, alignment, gap
points, step 2 reference). It needs `laspy[lazrs]`, numpy and scipy. All numbers are from the test recording of §6.3.

### 7.1 Easy GUI's step 1 run is the manual one

Easy GUI (output filter 1.0 m) against a manual step 1 run with preset 2 (1.5 m): the trajectory is **byte-identical**
(`session_poses.mrp`, `trajectory_lio_*.csv`). So step 1 is deterministic here, and the output filter is confirmed to be
output only. `scan_lio` has 89.33 M points instead of 84.21 M (+5.11 M, +6.1 %), the points between 1.0 and 1.5 m.

### 7.2 Step 1's points match step 2's export

Against `multi_view_tls_registration_step_2 <session.mjs> --export`: 89,326,756 points in both, max |ΔXYZ| = 0 (integer
coordinates identical), intensity and point_source_id identical. The trajectory CSV matches step 2's `trajectories.csv`
to 0.05 mm and 5e-10 in the quaternion.

`range_m` of step 1's points, compared with the raw range on coincident pairs: p99.9 difference 0.07 mm.

### 7.3 Near points line up with step 1's points

Test mode `--check-band 1.2` also exports the raw points with horizontal range 1.0–1.2 m, which step 1 exported too,
placed the near-point way and flagged `point_origin = 3`. Their nearest-neighbour distance to step 1's points:

- median 0.10 mm overall;
- for the 95.6 % that have a step 1 counterpart: p90 6.4 mm, p99 12.9 mm; 71 % land on the identical pose;
- the other 4.4 % lie in step 1's inter-chunk gaps, so they have no counterpart.

Rebuilding step 1's ramp from the raw files for scans 40–49 reproduces all 7,595,561 step 1 points (≤ 0.17 mm) and
shows that the whole difference comes from step 1's ramp time choosing a different node than the real time would:
|ramp − raw| p50 1.2 ms, p99 11.6 ms, max 17 ms, giving displacements of p90 9 mm, max 56 mm. Displacements of this
size have a negligible effect on sharpness and stem diameters: placing all points by raw time moves them by the same
mechanism and changes ground and stem thickness by less than 1 mm and diameters by a median of 0.4 mm (§8).

### 7.4 Gap points sit where a missing slice of step 1's points would

Per scan, against step 1's points of scans s−1 … s+1, gap points have a nearest-neighbour distance of median 19.4 mm,
p99 187 mm. Control: step 1's own points from the 5 ms before each gap, queried without them: median 19.8 mm, p99
192 mm. The same holds in every range bin (1–3 m: 10.6 vs 10.9 mm … 20+ m: 68.1 vs 68.2 mm).

277 gap points are within 0.5 µs of a pose time; classifying points from trajectory.csv (1 µs resolution) needs a
1 µs tolerance for them.

## 8. Experimental option: place every point by its raw time

`place_by_raw_time` (GUI advanced checkbox, `--place-by-raw-time`; default **off**) skips `scan_lio` and places every
raw point inside step 1's distance limits from the raw files, with the last node before its real gps_time (within
10 ms), and writes the raw gps_time. The near rule is unchanged.

On the test recording this gives 93,359,291 points inside step 1's limits instead of 89,326,756 (+4,026,808 from step
1's inter-chunk gaps, plus ~5.7 k first points of chunks).

**Displacement against the default.** Same raw point in both versions (matched via the rebuilt ramp; both placements
verified against the files to ≤ 0.17 mm): the same pose for 79.3 % of points, median displacement 0 in every range bin.
p95 is 12 / 20 / 34 / 69 mm at 1–3 / 3–5 / 5–10 / 10–20 m (moved points only: median 8 / 13 / 23 / 47 mm, p95
19 / 32 / 57 / 118 mm), which is a neighbouring node's rotation (~9 mrad).

**Sharpness** (2026-10-17; the same 89,326,756 matched points in both versions, selections made on the mean of both
positions, so the comparison is paired):

| Test | Default | Raw time | Paired difference (raw − default) by distance from the path | Raw time better in |
|---|---|---|---|---|
| Ground: 5,376 cells of 1 × 1 m within 20 m of the walk, lowest points in a ±0.15 m band, orthogonal plane fit, residual std | median 65.73 mm | 66.14 mm | +0.06 / +0.08 / +0.15 / +0.61 mm at 0–3 / 3–5 / 5–10 / 10–20 m | 37 % of cells (Wilcoxon p ≈ 1e-112) |
| Ground, ±0.05 m band | | | +0.05 … +0.24 mm | 41 % |
| Stems: 17 automatically detected, visually checked trunks, circle fit at 1.2–1.4 m above local ground, RMSE | median 19.25 mm | 19.31 mm | +0.05 / +0.43 / +0.03 / +0.17 mm at 1–3 / 3–5 / 5–10 / 10–20 m | 3 of 17 (p = 0.06) |

The ~66 mm ground thickness is mostly forest-floor roughness, which the paired design cancels. Stem diameters do not
differ systematically: median |ΔD| 0.4 mm (< 2 mm within 10 m; −3 … +7 mm for one-sided arcs at 10–20 m).

**Verdict:** the default (step 1's placement) is slightly sharper, by 0.1–0.6 mm on 20–60 mm thick surfaces. The most
likely reason is that step 1 optimised the trajectory with its own ramp-time points. Hence the option stays off. (The
ground and stem analysis scripts are not in the repository.)

## 9. Resources: time, memory, disk

Measured on the test recording (117 M raw points, 10-minute walk):

- Step 1 takes ~21 min with a peak memory footprint of 8.5 GB; the export ~2.5 min (143 s) with ~106 MB.
- Step 1 needs ~108 B per raw point of temporary disk at its peak (cache 7.5 GB + chunk caches ~4 GB + `scan_lio` for
  117 M points).
- Results are 2.14× the raw LAZ size: session 0.82× + point_cloud.laz 1.29× (with near and gap points) +
  trajectory.csv 0.03×.

The pre-flight check therefore estimates 120 B per raw point of temporary space plus 2.25× the raw LAZ size for results,
always reports the breakdown, warns below 1.5× the estimate and refuses below 1×.

## 10. Batch mode

"Several recordings" in the window and `--batch <parent folder> <output>` process many recordings one after another,
for example the plots of a field day overnight. Each recording runs through the unchanged `Pipeline` (pre-flight
re-check, staging, step 1 as a child process, export, clean-up), so its output folder is exactly that of a single run.

```
<output root>/
   batch.log               what happened, with times (flushed per line)
   batch_summary.csv       one row per recording and per skipped folder, rewritten after every recording
   <output name>/          one per recording: the files of §2 (+ easy_gui_done.txt once finished)
```

### 10.1 Finding recordings

`discoverRecordings` looks at the parent's direct subfolders, plus the parent itself (listed first, so a single
recording folder also works). A recording is a folder in which `classifyRecordingFiles` finds at least one point cloud
and one IMU file; the pre-flight check judges the rest. It is deliberately not recursive: a recording is the folder the
scanner wrote, and a deeper search would also find step 1's own `cache/`/`preview/` folders of old manual runs and
Easy GUI's staging links. Other subfolders are listed with a reason ("No IMU files", "Easy Forest HDMapping results,
not a recording", "Recordings are only looked for one folder level down"). Hidden and system folders (`.*`, `$*`,
`System Volume Information`) are left out silently.

### 10.2 Output names

The default is the recording's folder name, made unique ignoring ASCII case (the default file systems of macOS and
Windows are case-insensitive) by appending `_2`, `_3`, ... A parent that is a recording and contains a recording of
the same name is the common collision. A name is also passed over when its folder already holds files other than this
recording's finished result or an unfinished attempt. Edited names are validated for both systems (no
`/ \ : * ? " < > |` or control characters, no trailing dot or space, not `.`/`..`, not a Windows device name such as
`CON` or `COM1`) and must be unique among the recordings that will run.

The default output root is `Documents/Easy Forest HDMapping/<parent folder name>`. Unlike single mode it is reused when
it holds an earlier batch (it contains `batch_summary.csv`), so that choosing the same parent folder again continues
the batch.

The batch refuses an output root inside any recording (`batchOutputProblem`): `batch.log` and the summary would be
written there. The recordings' own output folders are covered by the per-recording check (§5.6).

### 10.3 Checks before the start

Every recording gets the single-mode pre-flight check against its own output folder, row by row on a background thread
in the window (the recordings may be on a slow USB stick). A recording that fails cannot be selected; one that is
already done (§10.4) is shown as such and skipped. The batch disk check needs

  the largest temporary need of any one selected recording + the results of all selected recordings,

because the recordings run one at a time and each frees its temporary space (§9) when it ends. Thresholds are those
of single mode: refused below the estimate, a warning below 1.5×. Disk space is checked again before every recording
(the pipeline repeats its pre-flight check), so a disk that fills up during the batch fails the affected recordings
with the usual message instead of crashing step 1.

### 10.4 Failures, done markers and restarting

- A recording that fails (check, step 1 or export) is marked failed with the pipeline's plain-language message, and the
  batch continues. Its output folder keeps what single mode keeps after a failure: the log, and the step 1 session if
  only the export failed.
- A recording that succeeds gets `easy_gui_done.txt`: version, recording path, a fingerprint, finish time, duration,
  raw and exported point counts. The fingerprint is the number of file pairs, the total size of the paired files and
  the first IMU timestamp. It identifies the recording without its path, because a USB stick can be mounted under
  another name or drive letter after a restart; the IMU timestamp separates equally large recordings.
- Restarting the same batch (same parent, same output root) skips recordings whose marker matches. Their summary rows
  say `already_done` and take duration and point counts from the marker.
- A recording whose output folder holds only Easy GUI's own run files (`easy_gui.log`, `work/`, `point_cloud.laz`,
  `trajectory.csv`, `README.txt`) but no marker is an unfinished attempt (failed, stopped, power cut). The batch runs
  the pipeline with `PipelineRequest::resume_unfinished`: the pre-flight check accepts the folder
  (`runPreflight(..., resume_unfinished)`), and the pipeline removes everything in it except `easy_gui.log`, which it
  continues. Any other file in the folder makes the check fail ("already contains files"), so nothing a user put
  there is ever deleted. Single mode keeps refusing non-empty folders.
- The summary is written to `batch_summary.csv.tmp` and renamed, after every recording, so it is never half-written
  and is current even if the batch never ends normally. On Windows a summary that is open in Excel cannot be
  replaced; the batch logs a warning and continues.

Summary columns: `recording, output_name, status, duration_s, raw_points, exported_points, output_path, error`.
`raw_points` counts all point records of the paired files (including no-return records), as the pre-flight check
does; `exported_points` is the size of `point_cloud.laz`. Statuses: `succeeded`, `failed`, `already_done`,
`cancelled` (stopped by "stop now"), `not_run` (the batch stopped before it), `excluded` (not selected, or its check
failed; the error column says which) and `skipped` (not a recording).

### 10.5 Running

- **Stop after current** lets the running recording finish; the rest become `not_run`. **Stop now** cancels the
  running recording exactly like single mode's cancel (child process killed, `work/` removed, log kept).
- **Sleep.** The batch thread holds a `SleepBlocker` for its whole run. On macOS this is an IOKit assertion
  `kIOPMAssertionTypePreventUserIdleSystemSleep` (shows up in `pmset -g assertions` as "Easy Forest HDMapping is
  processing recordings"; checked, and released when the batch ends); on Windows
  `SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)`, which applies to the calling thread, so it is set and cleared on the batch thread. Both
  prevent idle sleep only: the display may turn off, and closing a laptop lid still sleeps. Other platforms: no-op.
- **Progress and ETA.** Each recording is weighted by its raw point count. Within a recording, odometry counts for
  90 % and the export for 10 % (§9: ~21 min against ~2.5 min). The ETA is the processing rate of the recordings that
  really ran (already-done recordings take no time) applied to the remaining weight.
- `BatchRunner` drives an abstract `RecordingJob`; the real one wraps `Pipeline`, and the tests inject a fake one.

### 10.6 Tests

Unit tests (`tests/test_batch.cpp`, fake `RecordingJob`): discovery (sorting, nested folders, non-recordings, hidden
folders, parent that is a recording, with and without a trailing separator), output-name validation and collisions
(case, invalid characters, foreign / unfinished / finished folders), done-marker round trip and path independence,
batch disk arithmetic, output root inside a recording, failure-continues with summary and log contents, restart
skipping finished recordings and retrying an unfinished one, a folder filled after the check, stop after current and
stop now. `tests/test_preflight.cpp` covers `resume_unfinished` and keeps `holdsUnfinishedOutput` in sync with
`OutputLayout`.

Integration test (`EASY_GUI_TEST_RECORDING=mini4`): a parent with two copies of the recording (symlinked files), a
recording that passes the check but on which step 1 fails (one file pair cut to 200 points), and a non-recording
folder. Both copies succeed with identical point counts (2,566,166), the broken one fails with step 1's message, and a
restart skips the two copies and retries the broken one. 62 s on an Apple silicon laptop; one `mini4` copy takes
~30 s.

## 11. Status and next steps

- **M1 (done):** pre-flight check, staging, step 1 as a child process with progress and cancel.
- **M2 (done):** export (§6).
- **M3 (in progress):** validation script. `check_export.py` covers fields, range, alignment, gap points and the step 2
  reference. Still open:
  - Ramp-time error: for each odometry chunk, rebuild step 1's ramp from the raw points and compare with their real
    gps_time, reported twice: as step 1 does it (no-return points removed by the load filter *before* the ramp) and
    with those points kept, to quantify how much their removal contributes.
  - Near points: counts, range histogram, and sensor-frame density below 1.5 m.
- **M4 (done, released as 0.2.0):** batch mode (§10). The macOS side was run in the window and with
  `--batch` on `mini4` copies (stop after current, restart, sleep assertion). The Windows code (`SetThreadExecutionState`,
  Ctrl+C in `--batch`) compiles and its unit tests pass in CI (run 39163046329); not yet run on a real Windows PC.
- **v2:** gated pose-graph loop closure (§1).
- **Release builds:** `.github/workflows/easy-forest-{macos,windows}.yml`, on `easy-forest-v*` tags and manual
  dispatch; the packaging steps are scripts in `apps/easy_gui/packaging/` so they run locally the same way.
  - macOS (`macos-15` runner, Apple silicon, minimum 13.0): PROJ is built from source as a static library for 13.0
    (`build_proj.sh`; core_math links it for GNSS, which Easy GUI never calls; Homebrew's is built for the runner's
    macOS and drags in libtiff/curl). oneTBB without hwloc (`TBB_DISABLE_HWLOC_AUTOMATIC_SEARCH=ON`). OpenCV is not
    configured at all in `HDMAPPING_EASY_GUI_ONLY` builds (only the camera apps link it). `make_app.py` builds
    `Easy Forest HDMapping.app` (`easy_gui` main executable, `lidar_odometry_step_1` next to it in `Contents/MacOS`,
    `liblaszip`, `libtbb`, `libglut` in `Contents/Frameworks`, rpaths rewritten), re-signs it ad hoc and fails unless
    every bundled binary has minos <= 13.0 and references nothing outside the bundle, `/usr/lib` and `/System`. The
    build also fails on the linker's "built for newer macOS version" warning, which a static library would only show
    there. `make_dmg.sh` adds the Applications shortcut. Local and CI run: app 7.9 MB, DMG 3.6 MB; locally the bundled app
    processed `mini4` headless.
  - Windows (`windows-latest`, x64): `easy_gui.exe` is a WIN32-subsystem program (`/ENTRY:mainCRTStartup`, so `main`
    stays), `--headless`/`--export-only` attach to the parent console. A manifest sets UTF-8 as the process code page,
    because the file dialog, drag and drop and `ChildProcess` all use UTF-8. `HD_CPU_OPTIMIZATION=GENERIC`: `AUTO`
    adds `/arch:AVX2` from the runner's CPU. `make_zip.py` collects the DLLs with `dumpbin /dependents`, copies the
    MSVC runtime app-locally from VS's redist folder and fails on any DLL that is neither bundled nor part of Windows.
    First CI run: folder 7.1 MB (two exes, `laszip3`, `freeglut`, `tbb12` and the MSVC runtime DLLs), zip 3.3 MB.
    PROJ's DLL is not needed: nothing Easy GUI calls references it, so the linker drops the import.
  - `easy_gui_tests` cover `ChildProcess` on Windows too (cmd.exe; argument quoting through `--echo-args`).
  - No app icon yet (none exists for Easy GUI); no code signing or notarization (README explains the warnings).

Tests: `easy_gui_tests` (doctest, synthetic recordings and sessions). The integration tests against the real step 1
binary run only when `EASY_GUI_TEST_RECORDING=<recording folder>` is set; a folder of symlinks to the first 4 file
pairs of a recording is enough.
