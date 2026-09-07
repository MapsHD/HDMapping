#include "bow_loop_closure_detector.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <spdlog/spdlog.h>

#include <Core/point_cloud.h>
#include <Core/point_clouds.h>

namespace fs = std::filesystem;

namespace bow_loop_closure
{

namespace
{

// Parses "cam0_<timestamp_ns>.jpg" -> timestamp_ns. Same substring logic as
// TrajectoryViewer.cpp's loadImages(): strip the 5-char "cam0_" prefix and
// the 4-char ".jpg" suffix.
bool parseCam0Timestamp(const std::string& filename, int64_t& out_ts_ns)
{
    if (filename.rfind("cam0_", 0) != 0 || filename.size() <= 9)
        return false;
    try
    {
        out_ts_ns = std::stoll(filename.substr(5, filename.size() - 9));
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// Nearest entry in a vector<CameraFrame> sorted by timestamp_ns; -1 if
// outside max_gap_ns or frames is empty.
int nearestFrameIndex(const std::vector<CameraFrame>& frames, int64_t ts_ns, int64_t max_gap_ns)
{
    if (frames.empty())
        return -1;

    auto it = std::lower_bound(
        frames.begin(), frames.end(), ts_ns, [](const CameraFrame& f, int64_t ts) { return f.timestamp_ns < ts; });

    int best = -1;
    int64_t best_diff = std::numeric_limits<int64_t>::max();

    if (it != frames.end())
    {
        int idx = static_cast<int>(it - frames.begin());
        int64_t diff = std::llabs(frames[idx].timestamp_ns - ts_ns);
        if (diff < best_diff)
        {
            best_diff = diff;
            best = idx;
        }
    }
    if (it != frames.begin())
    {
        int idx = static_cast<int>(it - frames.begin()) - 1;
        int64_t diff = std::llabs(frames[idx].timestamp_ns - ts_ns);
        if (diff < best_diff)
        {
            best_diff = diff;
            best = idx;
        }
    }

    if (best >= 0 && best_diff <= max_gap_ns)
        return best;
    return -1;
}

// Detects ORB features + computes the fbow descriptor for one representative
// frame. Returns false (leaving outputs untouched) if the image can't be
// loaded or has no descriptors.
bool detectAndTransform(
    const std::string& image_path,
    cv::ORB& orb,
    fbow::Vocabulary& vocabulary,
    std::vector<cv::KeyPoint>& out_keypoints,
    cv::Mat& out_descriptors,
    fbow::fBow& out_bow)
{
    cv::Mat gray = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (gray.empty())
    {
        spdlog::warn("bow_loop_closure: could not read image '{}'", image_path);
        return false;
    }
    out_keypoints.clear();
    orb.detectAndCompute(gray, cv::noArray(), out_keypoints, out_descriptors);
    if (out_descriptors.empty())
        return false;
    out_bow = vocabulary.transform(out_descriptors);
    return true;
}

// Ratio-test ORB match + RANSAC fundamental matrix between two already-
// detected frames. Returns the inlier match count and (via out params) the
// matches actually kept as inliers, for drawing.
int verifyGeometry(
    const std::vector<cv::KeyPoint>& kp_i,
    const cv::Mat& desc_i,
    const std::vector<cv::KeyPoint>& kp_j,
    const cv::Mat& desc_j,
    const DetectorParams& params,
    std::vector<cv::DMatch>& out_inlier_matches)
{
    out_inlier_matches.clear();
    if (desc_i.empty() || desc_j.empty())
        return 0;

    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher.knnMatch(desc_i, desc_j, knn_matches, 2);

    std::vector<cv::DMatch> good_matches;
    for (const auto& m : knn_matches)
    {
        if (m.size() == 2 && m[0].distance < params.match_ratio_test * m[1].distance)
            good_matches.push_back(m[0]);
    }

    if (good_matches.size() < 8)
        return 0;

    std::vector<cv::Point2f> pts_i, pts_j;
    pts_i.reserve(good_matches.size());
    pts_j.reserve(good_matches.size());
    for (const auto& m : good_matches)
    {
        pts_i.push_back(kp_i[m.queryIdx].pt);
        pts_j.push_back(kp_j[m.trainIdx].pt);
    }

    std::vector<uchar> inlier_mask;
    cv::findFundamentalMat(
        pts_i, pts_j, cv::FM_RANSAC, params.ransac_reproj_threshold_px, params.ransac_confidence, inlier_mask);

    int inliers = 0;
    for (size_t k = 0; k < good_matches.size(); k++)
    {
        if (k < inlier_mask.size() && inlier_mask[k])
        {
            out_inlier_matches.push_back(good_matches[k]);
            inliers++;
        }
    }
    return inliers;
}

} // namespace

std::vector<CameraFrame> loadCameraFrames(const std::string& camera0_dir)
{
    std::vector<CameraFrame> frames;

    if (camera0_dir.empty() || !fs::is_directory(camera0_dir))
    {
        spdlog::warn("bow_loop_closure: CAMERA_0 directory not found: '{}'", camera0_dir);
        return frames;
    }

    for (const auto& entry : fs::directory_iterator(camera0_dir))
    {
        if (!entry.is_regular_file())
            continue;
        if (entry.path().extension() != ".jpg")
            continue;

        std::string name = entry.path().filename().string();
        int64_t ts_ns = 0;
        if (!parseCam0Timestamp(name, ts_ns))
            continue;

        frames.push_back(CameraFrame{ ts_ns, entry.path().string() });
    }

    std::sort(frames.begin(), frames.end(), [](const CameraFrame& a, const CameraFrame& b) { return a.timestamp_ns < b.timestamp_ns; });

    spdlog::info("bow_loop_closure: loaded {} camera frames from '{}'", frames.size(), camera0_dir);
    return frames;
}

std::vector<int> associateFramesToChunks(
    const PointClouds& point_clouds_container, const std::vector<CameraFrame>& frames, int64_t max_time_gap_ns)
{
    const auto& chunks = point_clouds_container.point_clouds;
    std::vector<int> chunk_to_frame(chunks.size(), -1);

    int matched = 0;
    for (size_t i = 0; i < chunks.size(); i++)
    {
        if (chunks[i].timestamps.empty())
            continue;
        int64_t chunk_ts_ns = static_cast<int64_t>(chunks[i].timestamps[0]);
        int idx = nearestFrameIndex(frames, chunk_ts_ns, max_time_gap_ns);
        chunk_to_frame[i] = idx;
        if (idx >= 0)
            matched++;
    }

    spdlog::info("bow_loop_closure: associated {}/{} chunks with a camera frame", matched, chunks.size());
    return chunk_to_frame;
}

std::string defaultVocabularyPath()
{
#ifdef FBOW_VOCABULARY_PATH
    return FBOW_VOCABULARY_PATH;
#else
    return "";
#endif
}

fbow::Vocabulary loadVocabulary(const std::string& vocabulary_path)
{
    fbow::Vocabulary vocabulary;
    if (vocabulary_path.empty() || !fs::exists(vocabulary_path))
        throw std::runtime_error("bow_loop_closure: vocabulary file not found: '" + vocabulary_path + "'");
    vocabulary.readFromFile(vocabulary_path);
    if (!vocabulary.isValid())
        throw std::runtime_error("bow_loop_closure: failed to load vocabulary from '" + vocabulary_path + "'");
    return vocabulary;
}

std::vector<Candidate> detectCandidates(
    const PointClouds& point_clouds_container,
    const std::vector<CameraFrame>& frames,
    const std::vector<int>& chunk_to_frame,
    const DetectorParams& params,
    fbow::Vocabulary& vocabulary)
{
    std::vector<Candidate> candidates;
    const auto& chunks = point_clouds_container.point_clouds;
    const size_t n = chunks.size();
    if (n == 0 || chunk_to_frame.size() != n)
        return candidates;

    // Cumulative trajectory arc-length from consecutive chunk poses.
    std::vector<double> arc_length(n, 0.0);
    for (size_t i = 1; i < n; i++)
    {
        double d = (chunks[i].m_pose.translation() - chunks[i - 1].m_pose.translation()).norm();
        arc_length[i] = arc_length[i - 1] + d;
    }

    // Precompute ORB keypoints/descriptors/BoW once per chunk that has an
    // associated frame (reused below for both the appearance score and the
    // geometric verification, within this one detectCandidates call).
    std::vector<std::vector<cv::KeyPoint>> keypoints_by_chunk(n);
    std::vector<cv::Mat> descriptors_by_chunk(n);
    std::vector<fbow::fBow> bow_by_chunk(n);
    std::vector<bool> has_bow(n, false);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(params.orb_features);
    for (size_t i = 0; i < n; i++)
    {
        int frame_idx = chunk_to_frame[i];
        if (frame_idx < 0 || frame_idx >= static_cast<int>(frames.size()))
            continue;
        has_bow[i] = detectAndTransform(
            frames[frame_idx].file_path, *orb, vocabulary, keypoints_by_chunk[i], descriptors_by_chunk[i], bow_by_chunk[i]);
    }

    for (size_t i = 0; i < n; i++)
    {
        if (!has_bow[i])
            continue;
        for (size_t j = i + 1; j < n; j++)
        {
            if (!has_bow[j])
                continue;

            // Stage 1: pose-gate.
            double arc_gap = arc_length[j] - arc_length[i];
            if (arc_gap < params.min_arc_gap_m)
                continue;

            Eigen::Vector3d ti = chunks[i].m_pose.translation();
            Eigen::Vector3d tj = chunks[j].m_pose.translation();
            double xy_dist = std::hypot(tj.x() - ti.x(), tj.y() - ti.y());
            if (xy_dist > params.max_xy_dist_m)
                continue;

            // Stage 2: appearance pre-filter.
            double score = fbow::fBow::score(bow_by_chunk[i], bow_by_chunk[j]);
            if (score < params.min_bow_score)
                continue;

            // Stage 3: geometric verification.
            std::vector<cv::DMatch> inlier_matches;
            int inliers = verifyGeometry(
                keypoints_by_chunk[i], descriptors_by_chunk[i], keypoints_by_chunk[j], descriptors_by_chunk[j], params, inlier_matches);
            if (inliers < params.min_inliers)
                continue;

            Candidate c;
            c.index_i = static_cast<int>(i);
            c.index_j = static_cast<int>(j);
            c.xy_dist_m = xy_dist;
            c.arc_gap_m = arc_gap;
            c.bow_score = score;
            c.inliers = inliers;
            candidates.push_back(c);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.inliers > b.inliers; });

    spdlog::info("bow_loop_closure: {} candidates passed pose-gate + appearance + geometric verification", candidates.size());
    return candidates;
}

cv::Mat buildCandidateMontage(
    const Candidate& candidate, const std::vector<CameraFrame>& frames, const std::vector<int>& chunk_to_frame, const DetectorParams& params)
{
    if (candidate.index_i < 0 || candidate.index_j < 0 || static_cast<size_t>(candidate.index_i) >= chunk_to_frame.size() ||
        static_cast<size_t>(candidate.index_j) >= chunk_to_frame.size())
        return cv::Mat();

    int frame_i = chunk_to_frame[candidate.index_i];
    int frame_j = chunk_to_frame[candidate.index_j];
    if (frame_i < 0 || frame_j < 0 || frame_i >= static_cast<int>(frames.size()) || frame_j >= static_cast<int>(frames.size()))
        return cv::Mat();

    cv::Mat color_i = cv::imread(frames[frame_i].file_path, cv::IMREAD_COLOR);
    cv::Mat color_j = cv::imread(frames[frame_j].file_path, cv::IMREAD_COLOR);
    if (color_i.empty() || color_j.empty())
        return cv::Mat();

    cv::Mat gray_i, gray_j;
    cv::cvtColor(color_i, gray_i, cv::COLOR_BGR2GRAY);
    cv::cvtColor(color_j, gray_j, cv::COLOR_BGR2GRAY);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(params.orb_features);
    std::vector<cv::KeyPoint> kp_i, kp_j;
    cv::Mat desc_i, desc_j;
    orb->detectAndCompute(gray_i, cv::noArray(), kp_i, desc_i);
    orb->detectAndCompute(gray_j, cv::noArray(), kp_j, desc_j);

    std::vector<cv::DMatch> inlier_matches;
    verifyGeometry(kp_i, desc_i, kp_j, desc_j, params, inlier_matches);

    cv::Mat montage;
    cv::drawMatches(
        color_i,
        kp_i,
        color_j,
        kp_j,
        inlier_matches,
        montage,
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return montage;
}

} // namespace bow_loop_closure
