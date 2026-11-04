#pragma once

// Automatic image-based loop-closure candidate detection for step 2, using
// the vendored fbow (Fast Bag-of-Words) library on ORB features from camera
// frames (mandeye CAMERA_0, or an Insta360's cam_front and cam_back). Framework-agnostic (no ImGui/raylib), so both
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

// One camera frame: `<prefix>_<timestamp_ns>.jpg` or `<timestamp_ns>.jpg`
// (mandeye's cam0_<ns>.jpg, Insta360 front_<ns>.jpg / back_<ns>.jpg), timestamp
// on the same clock as the LiDAR/IMU (same name rule as TrajectoryViewer.cpp's
// parseImageTsNs()).
struct CameraFrame
{
    int64_t timestamp_ns = 0;
    std::string file_path;
    int camera = 0; //!< index of its folder in loadCameraFrames' camera_dirs
};

struct DetectorParams
{
    // Stage 1 (pose-gate): keep chunk pairs far apart along the trajectory
    // arc-length but close in XY -- mirrors the standalone prototype's
    // --lc-min-arc-gap / --lc-max-xy-dist, computed here from poses already
    // in memory instead of a CSV sidecar.
    double min_arc_gap_m = 40.0;
    double max_xy_dist_m = 15.0;

    // Stage 2 (appearance pre-filter): fbow::fBow::score threshold. A cheap
    // pre-filter only: on a warehouse walk (mandeye CAMERA_0 and Insta360
    // front/back) the score barely separates revisits from far-apart pairs
    // (median 0.006-0.011 vs 0.004-0.006), and 0.02 kept fewer than 1 in 10
    // revisits. The inlier count below is what tells them apart.
    double min_bow_score = 0.005;

    // Stage 3 (geometric verification): ORB ratio-test match + RANSAC
    // fundamental matrix.
    int orb_features = 2000;
    float match_ratio_test = 0.75f;
    double ransac_reproj_threshold_px = 3.0;
    double ransac_confidence = 0.99;
    // Pairs more than 30 m apart (certainly not the same view) reach up to
    // ~28 inliers by chance on the same walk.
    int min_inliers = 30;

    // Images whose longer side exceeds this are downscaled to it before ORB
    // (0 = full resolution). Halving the Insta360 X5's 3840 px lens images
    // finds more revisits with fewer chance inliers, and twice as fast.
    int max_image_size_px = 1920;

    // Chunk<->camera-frame association tolerance (nearest-timestamp match).
    int64_t max_frame_time_gap_ns = 500000000; // 0.5 s
};

struct Candidate
{
    int index_i = -1;
    int index_j = -1;
    int frame_i = -1; //!< frames[] entry of chunk index_i that matched (the best camera pair)
    int frame_j = -1; //!< frames[] entry of chunk index_j that matched
    double xy_dist_m = 0.0;
    double arc_gap_m = 0.0;
    double bow_score = 0.0;
    int inliers = 0;
};

// Scans every folder in `camera_dirs` (one per camera, e.g. an Insta360's
// cam_front and cam_back) for `<prefix>_<timestamp_ns>.jpg` files. Returns
// them sorted by camera, then timestamp ascending.
std::vector<CameraFrame> loadCameraFrames(const std::vector<std::string>& camera_dirs);

// For each chunk in `point_clouds_container`, finds every camera's
// nearest-timestamp frame in `frames` (sorted as loadCameraFrames returns
// them). Returns one list of frame indices per chunk (same size as
// point_clouds_container.point_clouds), at most one per camera; a camera with
// no frame within `max_time_gap_ns` is left out, and a chunk with no
// per-point timestamp loaded gets an empty list.
std::vector<std::vector<int>> associateFramesToChunks(
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
// pipeline. `chunk_frames` is associateFramesToChunks()'s output. With several
// cameras, every camera pair of a chunk pair is tried (front vs back too, which
// finds a place revisited facing the other way) and the pair with the most
// inliers is kept. Returns surviving candidates sorted by inlier count
// descending.
std::vector<Candidate> detectCandidates(
    const PointClouds& point_clouds_container,
    const std::vector<CameraFrame>& frames,
    const std::vector<std::vector<int>>& chunk_frames,
    const DetectorParams& params,
    fbow::Vocabulary& vocabulary);

// Re-runs ORB detect + ratio-test match + RANSAC for one candidate's frame
// pair (independent of detectCandidates' internal per-run cache) and draws the
// inlier correspondences on a side-by-side image pair, for visual review
// before adding an edge. Returns an empty cv::Mat if either frame can't be
// loaded/matched.
cv::Mat buildCandidateMontage(const Candidate& candidate, const std::vector<CameraFrame>& frames, const DetectorParams& params);

} // namespace bow_loop_closure
