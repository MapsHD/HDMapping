#pragma once

#include <string>
#include <vector>

// Loads trajectories in the TUM RGB-D dataset format (one pose per line:
// "timestamp tx ty tz qx qy qz qw", space separated, '#'-prefixed lines
// treated as comments). Data-only -- unlike GNSS, this has no render()/GL
// dependency of its own; drawing it is the caller's job (see renderTUM()
// in multi_view_tls_registration_gui.cpp). tum_poses are drawn as GL_POINTS
// via ScanRenderer::uploadPoints()/drawPoints(ScanRenderer::PointsGPU&, ...)
// -- the same point-based approach and pointSize-uniform shader used for a
// scan's own local_trajectory (see PointCloud::line_width /
// ScanRenderer::drawTrajectories()) -- rather than a thin polyline, so
// point_size mirrors line_width's role: it's the drawn dot size, not a line
// width. version lets the caller cache its uploaded GPU buffer instead of
// re-uploading every frame -- see version's own comment below.
// show_correspondences still draws thin lines
// to each scan's nearest local_trajectory sample by timestamp -- matched
// against timestamps.first (the LIO trajectory CSV's "timestamp_nanoseconds"
// column, Unix-epoch nanoseconds), scaling TUM's own Unix-epoch-seconds
// timestamp up by 1e9 first. timestamps.second ("timestampUnix_nanoseconds")
// is the more obviously-named match but is commonly left at 0 (not every
// LIO run captures it), so it isn't used.
class TUM
{
public:
    struct TumPose
    {
        double timestamp;
        double x; // already Cartesian, in the trajectory's local/global frame
        double y;
        double z;
        double qx; // orientation quaternion
        double qy;
        double qz;
        double qw;
    };

    TUM() = default;
    ~TUM() = default;

    //! \brief Load trajectory data from TUM-format files
    //! \param input_file_names - vector of file names
    //! \param subtract_first_pose - if true, every pose is re-expressed relative to the
    //!        first (chronologically earliest) pose, so that pose becomes the identity
    //!        transform (0,0,0, no rotation) and the rest follow relative to it -- mirrors
    //!        GNSS's "load with offset -> move to (0,0,0)" localize option
    //! \return true if the data was loaded successfully, false otherwise
    bool load_data_from_tum(const std::vector<std::string>& input_file_names, bool subtract_first_pose = false);

    std::vector<TumPose> tum_poses;
    bool show_correspondences = false;
    float point_size = 4.0f; // GL_POINTS dot size (pixels) used to draw tum_poses

    // Bumped every time load_data_from_tum() succeeds. A GL-side cache keyed
    // on this (e.g. a renderer's own "last uploaded version") only needs to
    // re-upload tum_poses when this changes, rather than every frame.
    size_t version = 0;
};
