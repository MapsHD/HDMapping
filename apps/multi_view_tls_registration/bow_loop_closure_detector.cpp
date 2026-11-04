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

// Parses "<prefix>_<timestamp_ns>.jpg" or "<timestamp_ns>.jpg" ->
// timestamp_ns: the digits after the stem's last '_', same rule as
// TrajectoryViewer.cpp's parseImageTsNs(). The all-digits check rejects
// unrelated .jpgs, which would reach std::stoll.
bool parseFrameTimestamp(const fs::path& path, int64_t& out_ts_ns)
{
    if (path.extension() != ".jpg")
        return false;
    std::string stem = path.stem().string();
    if (const auto us = stem.rfind('_'); us != std::string::npos)
        stem = stem.substr(us + 1);
    if (stem.empty() || stem.find_first_not_of("0123456789") != std::string::npos)
        return false;
    try
    {
        out_ts_ns = std::stoll(stem);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

// Nearest entry of frames[begin, end), sorted by timestamp_ns; -1 if
// outside max_gap_ns or the range is empty.
int nearestFrameIndex(const std::vector<CameraFrame>& frames, int begin, int end, int64_t ts_ns, int64_t max_gap_ns)
{
    if (begin >= end)
        return -1;

    auto it = std::lower_bound(
        frames.begin() + begin, frames.begin() + end, ts_ns, [](const CameraFrame& f, int64_t ts) { return f.timestamp_ns < ts; });

    int best = -1;
    int64_t best_diff = std::numeric_limits<int64_t>::max();

    if (it != frames.begin() + end)
    {
        int idx = static_cast<int>(it - frames.begin());
        int64_t diff = std::llabs(frames[idx].timestamp_ns - ts_ns);
        if (diff < best_diff)
        {
            best_diff = diff;
            best = idx;
        }
    }
    if (it != frames.begin() + begin)
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

std::vector<CameraFrame> loadCameraFrames(const std::vector<std::string>& camera_dirs)
{
    std::vector<CameraFrame> frames;

    for (size_t camera = 0; camera < camera_dirs.size(); camera++)
    {
        const std::string& dir = camera_dirs[camera];
        if (dir.empty() || !fs::is_directory(dir))
        {
            spdlog::warn("bow_loop_closure: camera directory not found: '{}'", dir);
            continue;
        }

        size_t loaded = 0;
        for (const auto& entry : fs::directory_iterator(dir))
        {
            int64_t ts_ns = 0;
            if (!entry.is_regular_file() || !parseFrameTimestamp(entry.path(), ts_ns))
                continue;
            frames.push_back(CameraFrame{ ts_ns, entry.path().string(), static_cast<int>(camera) });
            loaded++;
        }
        spdlog::info("bow_loop_closure: loaded {} camera frames from '{}'", loaded, dir);
    }

    std::sort(
        frames.begin(),
        frames.end(),
        [](const CameraFrame& a, const CameraFrame& b)
        {
            return a.camera != b.camera ? a.camera < b.camera : a.timestamp_ns < b.timestamp_ns;
        });
    return frames;
}

std::vector<std::vector<int>> associateFramesToChunks(
    const PointClouds& point_clouds_container, const std::vector<CameraFrame>& frames, int64_t max_time_gap_ns)
{
    const auto& chunks = point_clouds_container.point_clouds;
    std::vector<std::vector<int>> chunk_frames(chunks.size());

    // [begin, end) of each camera's frames; loadCameraFrames sorts by camera first.
    std::vector<std::pair<int, int>> camera_ranges;
    for (int k = 0; k < static_cast<int>(frames.size()); k++)
    {
        if (k == 0 || frames[k].camera != frames[k - 1].camera)
            camera_ranges.emplace_back(k, k);
        camera_ranges.back().second = k + 1;
    }

    int matched = 0;
    for (size_t i = 0; i < chunks.size(); i++)
    {
        if (chunks[i].timestamps.empty())
            continue;
        int64_t chunk_ts_ns = static_cast<int64_t>(chunks[i].timestamps[0]);
        for (const auto& [begin, end] : camera_ranges)
        {
            int idx = nearestFrameIndex(frames, begin, end, chunk_ts_ns, max_time_gap_ns);
            if (idx >= 0)
                chunk_frames[i].push_back(idx);
        }
        if (!chunk_frames[i].empty())
            matched++;
    }

    spdlog::info(
        "bow_loop_closure: associated {}/{} chunks with a frame of {} camera(s)", matched, chunks.size(), camera_ranges.size());
    return chunk_frames;
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
    const std::vector<std::vector<int>>& chunk_frames,
    const DetectorParams& params,
    fbow::Vocabulary& vocabulary)
{
    std::vector<Candidate> candidates;
    const auto& chunks = point_clouds_container.point_clouds;
    const size_t n = chunks.size();
    if (n == 0 || chunk_frames.size() != n)
        return candidates;

    // Cumulative trajectory arc-length from consecutive chunk poses.
    std::vector<double> arc_length(n, 0.0);
    for (size_t i = 1; i < n; i++)
    {
        double d = (chunks[i].m_pose.translation() - chunks[i - 1].m_pose.translation()).norm();
        arc_length[i] = arc_length[i - 1] + d;
    }

    // Precompute ORB keypoints/descriptors/BoW once per frame some chunk is
    // associated with (reused below for both the appearance score and the
    // geometric verification, within this one detectCandidates call).
    const size_t m = frames.size();
    std::vector<std::vector<cv::KeyPoint>> keypoints_by_frame(m);
    std::vector<cv::Mat> descriptors_by_frame(m);
    std::vector<fbow::fBow> bow_by_frame(m);
    std::vector<bool> has_bow(m, false);

    cv::Ptr<cv::ORB> orb = cv::ORB::create(params.orb_features);
    for (const auto& frame_list : chunk_frames)
    {
        for (const int f : frame_list)
        {
            if (f < 0 || f >= static_cast<int>(m) || has_bow[f])
                continue;
            has_bow[f] =
                detectAndTransform(frames[f].file_path, *orb, vocabulary, keypoints_by_frame[f], descriptors_by_frame[f], bow_by_frame[f]);
        }
    }
    auto has_any_bow = [&](size_t chunk)
    {
        return std::any_of(chunk_frames[chunk].begin(), chunk_frames[chunk].end(), [&](int f) { return has_bow[f]; });
    };

    for (size_t i = 0; i < n; i++)
    {
        if (!has_any_bow(i))
            continue;
        for (size_t j = i + 1; j < n; j++)
        {
            if (!has_any_bow(j))
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

            // Stages 2 and 3 for every camera pair of the two chunks; the pair
            // with the most inliers wins.
            Candidate best;
            for (const int fi : chunk_frames[i])
            {
                for (const int fj : chunk_frames[j])
                {
                    if (!has_bow[fi] || !has_bow[fj])
                        continue;

                    // Stage 2: appearance pre-filter.
                    double score = fbow::fBow::score(bow_by_frame[fi], bow_by_frame[fj]);
                    if (score < params.min_bow_score)
                        continue;

                    // Stage 3: geometric verification.
                    std::vector<cv::DMatch> inlier_matches;
                    int inliers = verifyGeometry(
                        keypoints_by_frame[fi], descriptors_by_frame[fi], keypoints_by_frame[fj], descriptors_by_frame[fj], params, inlier_matches);
                    if (inliers < params.min_inliers || inliers <= best.inliers)
                        continue;

                    best.frame_i = fi;
                    best.frame_j = fj;
                    best.bow_score = score;
                    best.inliers = inliers;
                }
            }
            if (best.frame_i < 0)
                continue;

            best.index_i = static_cast<int>(i);
            best.index_j = static_cast<int>(j);
            best.xy_dist_m = xy_dist;
            best.arc_gap_m = arc_gap;
            candidates.push_back(best);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.inliers > b.inliers; });

    spdlog::info("bow_loop_closure: {} candidates passed pose-gate + appearance + geometric verification", candidates.size());
    return candidates;
}

cv::Mat buildCandidateMontage(const Candidate& candidate, const std::vector<CameraFrame>& frames, const DetectorParams& params)
{
    const int frame_i = candidate.frame_i;
    const int frame_j = candidate.frame_j;
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
