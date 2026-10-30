// Headless step 3: loads a project (.mjp) and its sessions, optionally proposes loop-closure edges between scans
// and registers them with ICP, optimizes the pose graph as the GUI's "Optimize GRAPH" does, and saves the session
// poses and/or the project. Nothing is written unless --save-poses or --save-project is given.

#include "multi_session_edges.h"
#include "multi_session_factor_graph.h"

#include <Core/session.h>
#include <Core/transformations.h>

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
    struct Options
    {
        std::string project;
        double propose_radius = 0.0; // 0: no edges proposed
        int min_scan_gap = 12;
        std::vector<double> icp_radii = { 1.0, 0.5, 0.25, 0.1 };
        int icp_iterations = 10;
        double min_fitness = 0.3;
        double max_correction_m = 0.5;
        double max_correction_deg = 2.0;
        double edge_weight = 1000000.0;
        int optimize_iterations = 0;
        bool save_poses = false;
        std::string save_project;
        double bucket = 0.1;
        bool verbose = false;
    };

    void print_usage(const char* argv0)
    {
        std::cout << "Step 3 (Multi session registration), headless " << HDMAPPING_CONFIGURE_PROJECT_VERSION << "\n\n"
                  << "USAGE:\n"
                  << fs::path(argv0).stem().string() << " --mjp <project.mjp> [options]\n\n"
                  << "Loads the project's sessions (ground-truth sessions first, as the GUI) and its edges, then in this order:\n"
                  << "  --propose-edges <m>        add edges between scans whose sensors are at most <m> apart in plan view: for\n"
                  << "                             every scan, the nearest scan of every session (within one session only scans\n"
                  << "                             --min-scan-gap apart); pairs already joined by an edge are skipped. Each\n"
                  << "                             proposed edge is registered with ICP and kept only when it passes the checks\n"
                  << "  --min-scan-gap <n>         default 12 (one scan is ~5 s of a step 1 session)\n"
                  << "  --icp-radii <r>[,<r>...]   ICP search radii of a proposed edge, in turn [m] (default 1,0.5,0.25,0.1)\n"
                  << "  --icp-iterations <n>       ICP iterations per radius (default 10)\n"
                  << "  --min-fitness <f>          keep an edge whose share of source points with a target point within the last\n"
                  << "                             radius is at least <f> (default 0.3)\n"
                  << "  --max-correction <m>,<deg> keep an edge that ICP moved at most this far from the current poses\n"
                  << "                             (default 0.5,2)\n"
                  << "  --edge-weight <w>          weight of every component of a proposed edge (default 1000000, as a manual edge)\n"
                  << "  --optimize <n>             run the pose graph optimization <n> times (the GUI's Optimize GRAPH runs 100)\n"
                  << "  --save-poses               overwrite session_poses.mrp of every session not marked ground truth\n"
                  << "  --save-project <out.mjp>   write the project with all its edges (the proposed ones included)\n"
                  << "Other options:\n"
                  << "  --bucket <m>               decimation at load, as the GUI's 'Downsample during load' (default 0.1; 0 = off)\n"
                  << "  --verbose                  keep the per-iteration output of ICP and of the optimization\n"
                  << "  -h, --help                 show this help and exit\n\n"
                  << "Exit codes: 0 done, 1 a project, session or output could not be read or written, 2 usage error.\n";
    }

    bool parse_doubles(const std::string& text, std::vector<double>& values)
    {
        values.clear();
        std::stringstream stream(text);
        std::string item;
        while (std::getline(stream, item, ','))
        {
            try
            {
                size_t used = 0;
                values.push_back(std::stod(item, &used));
                if (used != item.size())
                    return false;
            } catch (const std::exception&)
            {
                return false;
            }
        }
        return !values.empty();
    }

    // Returns -1 to go on, 0 after --help, 2 on a usage error.
    int parse_options(int argc, char* argv[], Options& options)
    {
        for (int i = 1; i < argc; i++)
        {
            const std::string arg = argv[i];
            if (arg == "-h" || arg == "--help" || arg == "/h" || arg == "/?")
            {
                print_usage(argv[0]);
                return 0;
            }
            if (arg == "--save-poses")
            {
                options.save_poses = true;
                continue;
            }
            if (arg == "--verbose")
            {
                options.verbose = true;
                continue;
            }
            if (i + 1 >= argc)
            {
                std::cerr << "missing value for " << arg << " (--help for usage)\n";
                return 2;
            }
            const std::string value = argv[++i];
            std::vector<double> numbers;
            const bool numeric = parse_doubles(value, numbers);
            const auto need = [&](size_t count) -> bool
            {
                if (numeric && numbers.size() == count)
                    return true;
                std::cerr << "bad value for " << arg << ": '" << value << "' (--help for usage)\n";
                return false;
            };

            if (arg == "--mjp")
                options.project = value;
            else if (arg == "--save-project")
                options.save_project = value;
            else if (arg == "--propose-edges")
            {
                if (!need(1) || numbers[0] <= 0.0)
                    return 2;
                options.propose_radius = numbers[0];
            }
            else if (arg == "--min-scan-gap")
            {
                if (!need(1) || numbers[0] < 1.0)
                    return 2;
                options.min_scan_gap = static_cast<int>(numbers[0]);
            }
            else if (arg == "--icp-radii")
            {
                if (!numeric ||
                    std::any_of(
                        numbers.begin(),
                        numbers.end(),
                        [](double r)
                        {
                            return r <= 0.0;
                        }))
                {
                    std::cerr << "bad value for " << arg << ": '" << value << "' (--help for usage)\n";
                    return 2;
                }
                options.icp_radii = numbers;
            }
            else if (arg == "--icp-iterations")
            {
                if (!need(1) || numbers[0] < 1.0)
                    return 2;
                options.icp_iterations = static_cast<int>(numbers[0]);
            }
            else if (arg == "--min-fitness")
            {
                if (!need(1))
                    return 2;
                options.min_fitness = numbers[0];
            }
            else if (arg == "--max-correction")
            {
                if (!need(2))
                    return 2;
                options.max_correction_m = numbers[0];
                options.max_correction_deg = numbers[1];
            }
            else if (arg == "--edge-weight")
            {
                if (!need(1) || numbers[0] <= 0.0)
                    return 2;
                options.edge_weight = numbers[0];
            }
            else if (arg == "--optimize")
            {
                if (!need(1) || numbers[0] < 0.0)
                    return 2;
                options.optimize_iterations = static_cast<int>(numbers[0]);
            }
            else if (arg == "--bucket")
            {
                if (!need(1) || numbers[0] < 0.0)
                    return 2;
                options.bucket = numbers[0];
            }
            else
            {
                std::cerr << "unknown option " << arg << " (--help for usage)\n";
                return 2;
            }
        }
        if (options.project.empty())
        {
            std::cerr << "--mjp <project.mjp> is required (--help for usage)\n";
            return 2;
        }
        return -1;
    }

    // The project file as the GUI reads and writes it: session file names and loop-closure edges.
    bool load_project(const std::string& file_name, std::vector<std::string>& session_file_names, std::vector<Edge>& edges)
    {
        try
        {
            std::ifstream file(file_name);
            if (!file.good())
            {
                std::cerr << "cannot read project '" << file_name << "'\n";
                return false;
            }
            const nlohmann::json data = nlohmann::json::parse(file);

            session_file_names.clear();
            for (const auto& fn_json : data.at("session_file_names"))
                session_file_names.push_back(fn_json.at("session_file_name").get<std::string>());

            edges.clear();
            for (const auto& edge_json : data.value("loop_closure_edges", nlohmann::json::array()))
            {
                Edge edge;
                edge.index_from = edge_json.at("index_from");
                edge.index_to = edge_json.at("index_to");
                edge.index_session_from = edge_json.at("index_session_from");
                edge.index_session_to = edge_json.at("index_session_to");
                edge.is_fixed_px = edge_json.value("is_fixed_px", false);
                edge.is_fixed_py = edge_json.value("is_fixed_py", false);
                edge.is_fixed_pz = edge_json.value("is_fixed_pz", false);
                edge.is_fixed_om = edge_json.value("is_fixed_om", false);
                edge.is_fixed_fi = edge_json.value("is_fixed_fi", false);
                edge.is_fixed_ka = edge_json.value("is_fixed_ka", false);
                edge.relative_pose_tb.px = edge_json.at("px");
                edge.relative_pose_tb.py = edge_json.at("py");
                edge.relative_pose_tb.pz = edge_json.at("pz");
                edge.relative_pose_tb.om = edge_json.at("om");
                edge.relative_pose_tb.fi = edge_json.at("fi");
                edge.relative_pose_tb.ka = edge_json.at("ka");
                edge.relative_pose_tb_weights.px = edge_json.at("w_px");
                edge.relative_pose_tb_weights.py = edge_json.at("w_py");
                edge.relative_pose_tb_weights.pz = edge_json.at("w_pz");
                edge.relative_pose_tb_weights.om = edge_json.at("w_om");
                edge.relative_pose_tb_weights.fi = edge_json.at("w_fi");
                edge.relative_pose_tb_weights.ka = edge_json.at("w_ka");
                edges.push_back(edge);
            }
        } catch (const std::exception& e)
        {
            std::cerr << "cannot read project '" << file_name << "': " << e.what() << "\n";
            return false;
        }
        std::cout << "project '" << file_name << "': " << session_file_names.size() << " sessions, " << edges.size() << " edges\n";
        return true;
    }

    bool save_project(const std::string& file_name, const std::vector<std::string>& session_file_names, const std::vector<Edge>& edges)
    {
        nlohmann::json jj;
        nlohmann::json jsession_file_names = nlohmann::json::array();
        for (const auto& name : session_file_names)
            jsession_file_names.push_back({ { "session_file_name", name } });
        jj["session_file_names"] = jsession_file_names;

        nlohmann::json jloop_closure_edges = nlohmann::json::array();
        for (const auto& edge : edges)
        {
            jloop_closure_edges.push_back(
                {
                    { "px", edge.relative_pose_tb.px },
                    { "py", edge.relative_pose_tb.py },
                    { "pz", edge.relative_pose_tb.pz },
                    { "om", edge.relative_pose_tb.om },
                    { "fi", edge.relative_pose_tb.fi },
                    { "ka", edge.relative_pose_tb.ka },
                    { "w_px", edge.relative_pose_tb_weights.px },
                    { "w_py", edge.relative_pose_tb_weights.py },
                    { "w_pz", edge.relative_pose_tb_weights.pz },
                    { "w_om", edge.relative_pose_tb_weights.om },
                    { "w_fi", edge.relative_pose_tb_weights.fi },
                    { "w_ka", edge.relative_pose_tb_weights.ka },
                    { "index_from", edge.index_from },
                    { "index_to", edge.index_to },
                    { "is_fixed_px", edge.is_fixed_px },
                    { "is_fixed_py", edge.is_fixed_py },
                    { "is_fixed_pz", edge.is_fixed_pz },
                    { "is_fixed_om", edge.is_fixed_om },
                    { "is_fixed_fi", edge.is_fixed_fi },
                    { "is_fixed_ka", edge.is_fixed_ka },
                    { "index_session_from", edge.index_session_from },
                    { "index_session_to", edge.index_session_to },
                });
        }
        jj["loop_closure_edges"] = jloop_closure_edges;

        std::ofstream file(file_name);
        if (!file.good())
        {
            std::cerr << "cannot write project '" << file_name << "'\n";
            return false;
        }
        file << jj.dump(2);
        file.close();
        if (!file)
        {
            std::cerr << "cannot write project '" << file_name << "'\n";
            return false;
        }
        std::cout << "saved project '" << file_name << "': " << edges.size() << " edges\n";
        return true;
    }

    // Loads the sessions as the GUI does, ground-truth sessions first; the edges' session indices follow the new order.
    bool load_sessions(
        std::vector<std::string>& session_file_names, std::vector<Edge>& edges, double bucket, std::vector<Session>& sessions)
    {
        std::vector<Session> loaded(session_file_names.size());
        for (size_t i = 0; i < session_file_names.size(); i++)
        {
            if (!loaded[i].load(fs::path(session_file_names[i]).string(), bucket > 0.0, bucket, bucket, bucket, false))
            {
                std::cerr << "cannot load session '" << session_file_names[i] << "'\n";
                return false;
            }
        }

        std::vector<size_t> order;
        for (size_t i = 0; i < loaded.size(); i++)
            if (loaded[i].is_ground_truth)
                order.push_back(i);
        for (size_t i = 0; i < loaded.size(); i++)
            if (!loaded[i].is_ground_truth)
                order.push_back(i);

        std::map<int, int> new_index;
        std::vector<std::string> names;
        sessions.clear();
        for (size_t k = 0; k < order.size(); k++)
        {
            new_index[static_cast<int>(order[k])] = static_cast<int>(k);
            names.push_back(session_file_names[order[k]]);
            sessions.push_back(std::move(loaded[order[k]]));
        }
        session_file_names = names;

        for (auto& e : edges)
        {
            if (!new_index.contains(e.index_session_from) || !new_index.contains(e.index_session_to))
            {
                std::cerr << "edge names session " << e.index_session_from << " -> " << e.index_session_to << ", the project has "
                          << sessions.size() << "\n";
                return false;
            }
            e.index_session_from = new_index[e.index_session_from];
            e.index_session_to = new_index[e.index_session_to];
            const int scans_from = static_cast<int>(sessions[e.index_session_from].point_clouds_container.point_clouds.size());
            const int scans_to = static_cast<int>(sessions[e.index_session_to].point_clouds_container.point_clouds.size());
            if (e.index_from < 0 || e.index_from >= scans_from || e.index_to < 0 || e.index_to >= scans_to)
            {
                std::cerr << "edge names a scan the session does not have: " << e.index_from << " -> " << e.index_to << "\n";
                return false;
            }
        }
        return true;
    }

    // Discards std::cout while alive: ICP and the optimization print every iteration.
    class QuietCout
    {
    public:
        explicit QuietCout(bool quiet)
            : m_old(quiet ? std::cout.rdbuf(&m_null) : nullptr)
        {
        }
        ~QuietCout()
        {
            if (m_old != nullptr)
                std::cout.rdbuf(m_old);
        }
        QuietCout(const QuietCout&) = delete;
        QuietCout& operator=(const QuietCout&) = delete;

    private:
        struct NullBuffer : std::streambuf
        {
            int overflow(int c) override
            {
                return c;
            }
        };
        NullBuffer m_null;
        std::streambuf* m_old;
    };

    double percentile(std::vector<double> values, double q)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        return values[static_cast<size_t>(q * (values.size() - 1) + 0.5)];
    }

    void propose_and_register(const Options& options, const std::vector<Session>& sessions, std::vector<Edge>& edges)
    {
        EdgeProposalSettings settings;
        settings.radius = options.propose_radius;
        settings.min_scan_gap = options.min_scan_gap;
        settings.weights = { options.edge_weight, options.edge_weight, options.edge_weight,
                             options.edge_weight, options.edge_weight, options.edge_weight };
        std::vector<Edge> proposed = propose_edges(sessions, edges, settings);
        std::cout << "proposed " << proposed.size() << " edges (radius " << options.propose_radius << " m, min scan gap "
                  << options.min_scan_gap << ")\n";

        int failed = 0;
        int low_fitness = 0;
        int moved_too_far = 0;
        std::vector<double> corrections_m;
        std::map<std::pair<int, int>, int> per_session_pair;
        std::vector<Edge> accepted;
        for (size_t i = 0; i < proposed.size(); i++)
        {
            Edge edge = proposed[i];
            const Eigen::Affine3d m_before = affine_matrix_from_pose_tait_bryan(edge.relative_pose_tb);
            EdgeQuality quality;
            bool ok = false;
            {
                QuietCout quiet(!options.verbose);
                ok = icp_edge(sessions, edge, options.icp_radii, options.icp_iterations, 0, 0, &quality);
            }
            if ((i + 1) % 100 == 0 || i + 1 == proposed.size())
                std::cout << "registered " << i + 1 << " of " << proposed.size() << " edges\n";
            if (!ok)
            {
                failed++;
                continue;
            }

            const Eigen::Affine3d correction = m_before.inverse() * affine_matrix_from_pose_tait_bryan(edge.relative_pose_tb);
            const double correction_m = correction.translation().norm();
            const double correction_deg = Eigen::AngleAxisd(correction.rotation()).angle() * 180.0 / M_PI;
            if (quality.fitness < options.min_fitness)
            {
                low_fitness++;
                continue;
            }
            if (correction_m > options.max_correction_m || correction_deg > options.max_correction_deg)
            {
                moved_too_far++;
                continue;
            }
            corrections_m.push_back(correction_m);
            per_session_pair[{ edge.index_session_from, edge.index_session_to }]++;
            accepted.push_back(edge);
        }

        std::cout << "kept " << accepted.size() << " of " << proposed.size() << " proposed edges; dropped: " << failed << " ICP failed, "
                  << low_fitness << " fitness below " << options.min_fitness << ", " << moved_too_far << " moved more than "
                  << options.max_correction_m << " m or " << options.max_correction_deg << " deg\n";
        if (!accepted.empty())
        {
            std::cout << "ICP correction of the kept edges: median " << percentile(corrections_m, 0.5) * 1000.0 << " mm, p90 "
                      << percentile(corrections_m, 0.9) * 1000.0 << " mm\n";
            for (const auto& [pair, count] : per_session_pair)
                std::cout << "  session " << pair.first << " -> " << pair.second << ": " << count << " edges\n";
        }
        edges.insert(edges.end(), accepted.begin(), accepted.end());
    }

    void optimize_graph(const Options& options, std::vector<Session>& sessions, const std::vector<Edge>& edges)
    {
        std::vector<std::vector<Eigen::Affine3d>> before;
        for (const auto& s : sessions)
        {
            before.emplace_back();
            for (const auto& pc : s.point_clouds_container.point_clouds)
                before.back().push_back(pc.m_pose);
        }

        // as the GUI's default motion model weights (1 sigma: m, deg)
        const TaitBryanPose motion_model_weights = { 0.01, 0.01, 0.01, 0.1, 0.1, 0.1 };
        for (int i = 0; i < options.optimize_iterations; i++)
        {
            QuietCout quiet(!options.verbose);
            optimize(sessions, edges, motion_model_weights);
        }

        std::cout << "optimized the pose graph: " << options.optimize_iterations << " iterations, " << edges.size() << " edges\n";
        for (size_t s = 0; s < sessions.size(); s++)
        {
            std::vector<double> moved;
            const auto& point_clouds = sessions[s].point_clouds_container.point_clouds;
            for (size_t k = 0; k < point_clouds.size(); k++)
                moved.push_back((point_clouds[k].m_pose.translation() - before[s][k].translation()).norm());
            std::cout << "  session " << s << ": scans moved median " << percentile(moved, 0.5) * 1000.0 << " mm, max "
                      << percentile(moved, 1.0) * 1000.0 << " mm\n";
        }
    }

    bool save_poses(std::vector<Session>& sessions)
    {
        bool ok = true;
        for (auto& session : sessions)
        {
            if (session.is_ground_truth || Session::is_point_cloud_file(session.session_file_name))
                continue;
            const std::string file_name = fs::path(session.point_clouds_container.poses_file_name).string();
            if (session.point_clouds_container.save_poses(file_name, false))
                std::cout << "saved poses '" << file_name << "'\n";
            else
            {
                std::cerr << "cannot write poses '" << file_name << "'\n";
                ok = false;
            }
        }
        return ok;
    }
} // namespace

int main(int argc, char* argv[])
{
    Options options;
    const int parsed = parse_options(argc, argv, options);
    if (parsed >= 0)
        return parsed;

    std::vector<std::string> session_file_names;
    std::vector<Edge> edges;
    if (!load_project(options.project, session_file_names, edges))
        return 1;

    std::vector<Session> sessions;
    {
        QuietCout quiet(!options.verbose);
        if (!load_sessions(session_file_names, edges, options.bucket, sessions))
            return 1;
    }
    for (const auto& s : sessions)
        std::cout << "session '" << s.session_file_name << "': " << s.point_clouds_container.point_clouds.size() << " scans, ground truth ["
                  << int(s.is_ground_truth) << "]\n";

    if (options.propose_radius > 0.0)
        propose_and_register(options, sessions, edges);

    if (options.optimize_iterations > 0)
        optimize_graph(options, sessions, edges);

    bool ok = true;
    if (options.save_poses)
        ok = save_poses(sessions) && ok;
    if (!options.save_project.empty())
        ok = save_project(options.save_project, session_file_names, edges) && ok;
    if (!options.save_poses && options.save_project.empty())
        std::cout << "nothing written (no --save-poses or --save-project)\n";
    return ok ? 0 : 1;
}
