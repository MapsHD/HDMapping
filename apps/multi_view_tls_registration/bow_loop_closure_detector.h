#pragma once

// Automatic image-based loop-closure candidate detection for step 2, using
// the vendored fbow (Fast Bag-of-Words) library on ORB features from mandeye
// CAMERA_0 frames. Framework-agnostic (no ImGui/raylib), so both
// bow_loop_closure_gui() (multi_view_tls_registration_gui.cpp) and the
// headless bow_loop_closure_detect console tool can drive it.
//
// Design follows BOW_PLACE_RECOGNITION_HANDOFF.md's lesson from a prior
// standalone prototype: appearance-only matching had ~0% precision on a
// repetitive scene, so candidates are gated by trajectory pose proximity
// (far apart in arc-length, close in XY) *before* appearance is trusted, and
// appearance + RANSAC geometric verification only *confirm* a pose-gated
// candidate -- they never discover one on their own. Detected candidates are
// meant for manual review (see the GUI's montage preview), not auto-added to
// the pose graph.

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include <fbow.h>

class PointClouds;

namespace bow_loop_closure
{

// One CAMERA_0 frame: `cam0_<timestamp_ns>.jpg`, timestamp on the same clock
// as the LiDAR/IMU (same parsing as TrajectoryViewer.cpp's loadImages()).
struct CameraFrame
{
    int64_t timestamp_ns = 0;
    std::string file_path;
};

struct DetectorParams
{
    // Stage 1 (pose-gate): keep chunk pairs far apart along the trajectory
    // arc-length but close in XY -- mirrors the standalone prototype's
    // --lc-min-arc-gap / --lc-max-xy-dist, computed here from poses already
    // in memory instead of a CSV sidecar.
    double min_arc_gap_m = 40.0;
    double max_xy_dist_m = 15.0;

    // Stage 2 (appearance pre-filter): fbow::fBow::score threshold.
    double min_bow_score = 0.02;

    // Stage 3 (geometric verification): ORB ratio-test match + RANSAC
    // fundamental matrix.
    int orb_features = 2000;
    float match_ratio_test = 0.75f;
    double ransac_reproj_threshold_px = 3.0;
    double ransac_confidence = 0.99;
    int min_inliers = 15;

    // Chunk<->camera-frame association tolerance (nearest-timestamp match).
    int64_t max_frame_time_gap_ns = 500000000; // 0.5 s
};

struct Candidate
{
    int index_i = -1;
    int index_j = -1;
    double xy_dist_m = 0.0;
    double arc_gap_m = 0.0;
    double bow_score = 0.0;
    int inliers = 0;
};

// Scans `camera0_dir` for `cam0_<timestamp_ns>.jpg` files, sorted by
// timestamp ascending.
std::vector<CameraFrame> loadCameraFrames(const std::string& camera0_dir);

// For each chunk in `point_clouds_container`, finds the nearest-timestamp
// frame in `frames` (frames must be sorted by timestamp_ns, as returned by
// loadCameraFrames). Returns one frame index per chunk (same size as
// point_clouds_container.point_clouds), or -1 where no frame falls within
// `max_time_gap_ns`, or the chunk has no per-point timestamp loaded.
std::vector<int> associateFramesToChunks(
    const PointClouds& point_clouds_container, const std::vector<CameraFrame>& frames, int64_t max_time_gap_ns);

// Absolute path to the fbow ORB vocabulary bundled with this build
// (3rdparty/fbow/vocabularies/orb_mur.fbow), baked in at configure time via
// the FBOW_VOCABULARY_PATH compile definition. This is a source-tree path,
// fine for local/dev builds; a packaged install would instead want to copy
// the file next to the binary and resolve it relative to argv[0].
std::string defaultVocabularyPath();

// Throws std::runtime_error if the vocabulary file can't be read.
fbow::Vocabulary loadVocabulary(const std::string& vocabulary_path);

// Runs the full pose-gate -> appearance-filter -> geometric-verification
// pipeline. `chunk_to_frame` is associateFramesToChunks()'s output. Returns
// surviving candidates sorted by inlier count descending.
std::vector<Candidate> detectCandidates(
    const PointClouds& point_clouds_container,
    const std::vector<CameraFrame>& frames,
    const std::vector<int>& chunk_to_frame,
    const DetectorParams& params,
    fbow::Vocabulary& vocabulary);

// Re-runs ORB detect + ratio-test match + RANSAC for one candidate pair
// (independent of detectCandidates' internal per-run cache) and draws the
// inlier correspondences on a side-by-side image pair, for visual review
// before adding an edge. Returns an empty cv::Mat if either frame can't be
// loaded/matched.
cv::Mat buildCandidateMontage(
    const Candidate& candidate,
    const std::vector<CameraFrame>& frames,
    const std::vector<int>& chunk_to_frame,
    const DetectorParams& params);

} // namespace bow_loop_closure
