#pragma once

// Shrinks/repositions the just-created window so it fits within the current
// monitor's usable area, DPI-aware. Was duplicated byte-for-byte across
// apps/camera_lidar_calibration, apps/camera_lidar_trajectory_viewer and
// apps/camera_lidar_intrinsics_calib; multi_view_tls_registration_step_2 had
// its own inline version without the DPI-scale correction (fine on a 1x
// display, but centers/sizes wrong on a 2x Retina one).
namespace raylib_widgets {

// marginW/marginH: side/top+bottom breathing room (OS menu bar, title bar,
// dock) subtracted from the monitor's usable area.
// centerVertically: false positions the window near the top (Y=30, the
// calib apps' behavior); true centers it vertically (step2's behavior).
void fitWindowToScreen(int marginW = 40, int marginH = 100, bool centerVertically = false);

} // namespace raylib_widgets
