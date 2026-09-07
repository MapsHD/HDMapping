// Headless verification tool for the automatic image-based (fbow) loop
// closure detector: loads a step-2 session + one or more camera folders, runs
// the pose-gate -> appearance -> geometric-verification pipeline, and prints
// candidate loop-closure pairs as CSV. See
// apps/multi_view_tls_registration/bow_loop_closure_detector.h for the
// pipeline itself (shared with the GUI) and BOW_PLACE_RECOGNITION_HANDOFF.md
// for why it's shaped this way.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

#include <spdlog/spdlog.h>

#include <Core/session.h>

#include "../multi_view_tls_registration/bow_loop_closure_detector.h"

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cout << "USAGE: " << argv[0] << " session_file(.mjs/.json) camera_dir [camera_dir...] [output.csv]" << std::endl;
        std::cout << "  session_file : step 2 session file (e.g. .../lio_result_100/session.mjs)" << std::endl;
        std::cout << "  camera_dir   : folder of <prefix>_<timestamp_ns>.jpg frames, e.g. mandeye CAMERA_0 or an" << std::endl;
        std::cout << "                 Insta360's cam_front and cam_back; several folders are matched together" << std::endl;
        std::cout << "  output.csv   : optional (last argument, ending in .csv); if omitted, prints CSV to stdout" << std::endl;
        return 1;
    }

    std::string session_file = argv[1];
    std::vector<std::string> camera_dirs(argv + 2, argv + argc);
    std::string output_csv;
    if (camera_dirs.size() > 1 && std::filesystem::path(camera_dirs.back()).extension() == ".csv")
    {
        output_csv = camera_dirs.back();
        camera_dirs.pop_back();
    }

    Session session;
    if (!session.load(session_file, false, 0.0, 0.0, 0.0, false))
    {
        spdlog::error("Failed to load session '{}'", session_file);
        return 2;
    }
    spdlog::info("Loaded session with {} chunks", session.point_clouds_container.point_clouds.size());

    bow_loop_closure::DetectorParams params;

    auto frames = bow_loop_closure::loadCameraFrames(camera_dirs);
    if (frames.empty())
    {
        spdlog::error("No camera frames found");
        return 3;
    }

    auto chunk_frames = bow_loop_closure::associateFramesToChunks(session.point_clouds_container, frames, params.max_frame_time_gap_ns);

    std::string vocab_path = bow_loop_closure::defaultVocabularyPath();
    std::unique_ptr<fbow::Vocabulary> vocabulary;
    try
    {
        vocabulary = std::make_unique<fbow::Vocabulary>(bow_loop_closure::loadVocabulary(vocab_path));
    }
    catch (const std::exception& e)
    {
        spdlog::error("{}", e.what());
        return 4;
    }

    auto candidates = bow_loop_closure::detectCandidates(session.point_clouds_container, frames, chunk_frames, params, *vocabulary);

    std::ostream* out = &std::cout;
    std::ofstream file_out;
    if (!output_csv.empty())
    {
        file_out.open(output_csv);
        if (!file_out.good())
        {
            spdlog::error("Could not open output file '{}'", output_csv);
            return 5;
        }
        out = &file_out;
    }

    *out << "index_i,index_j,xy_dist_m,arc_gap_m,bow_score,inliers,image_i,image_j" << std::endl;
    for (const auto& c : candidates)
    {
        *out << c.index_i << "," << c.index_j << "," << c.xy_dist_m << "," << c.arc_gap_m << "," << c.bow_score << "," << c.inliers << ","
             << frames[c.frame_i].file_path << "," << frames[c.frame_j].file_path << std::endl;
    }

    spdlog::info(
        "{} candidates ({} chunks pose-gated by arc_gap>={}, xy_dist<={}; {}/{} chunks had an associated camera frame)",
        candidates.size(),
        session.point_clouds_container.point_clouds.size(),
        params.min_arc_gap_m,
        params.max_xy_dist_m,
        static_cast<int>(std::count_if(chunk_frames.begin(), chunk_frames.end(), [](const auto& f) { return !f.empty(); })),
        chunk_frames.size());

    return 0;
}
