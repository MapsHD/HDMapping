#include "Core/color_las_loader.h"
#include "Core/pair_wise_iterative_closest_point.h"
#include "Core/session.h"
#include "Core/transformations.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace
{
    enum class Method
    {
        PointToPoint,
        PointToPlane,
        MonteCarlo
    };

    //! Seconds as h:mm:ss.
    std::string format_duration(double seconds)
    {
        const long long total = static_cast<long long>(seconds + 0.5);
        std::ostringstream out;
        out << total / 3600 << ':' << std::setw(2) << std::setfill('0') << (total / 60) % 60 << ':' << std::setw(2) << std::setfill('0')
            << total % 60;
        return out.str();
    }

    void print_usage(const char* program)
    {
        std::cerr << "Usage: " << program << " <TLS(laz format)> <session_data> <output_file> [options]\n"
                  << "Options:\n"
                  << "  --method point_to_point|point_to_plane|monte_carlo  uncertainty method (default point_to_point)\n"
                  << "  --search-radius <m>       ICP search radius (default 0.3)\n"
                  << "  --runs <n>                monte_carlo: ICP runs per node (default 20)\n"
                  << "  --start-offset-m <m>      monte_carlo: start offset on px, py, pz (default 0.05)\n"
                  << "  --start-offset-deg <deg>  monte_carlo: start offset on om, fi, ka (default 0.5)\n"
                  << "  --node-step <n>           compute every n-th trajectory node of each chunk (default 1: all)\n"
                  << "  --decimation <m>          voxel size the session points are decimated to (default 0: off)\n"
                  << "  --tls-decimation <m>      voxel size the TLS points are decimated to (default 0: off)" << std::endl;
    }
} // namespace

int main(int argc, char* argv[])
{
    std::cout << "Session Uncertainty Calculation" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "This program calculates the uncertainty of a session for each trajectory node from inverse ICP/Hessian assuming having "
                 "TLS ground truth."
              << std::endl;

    if (argc < 4)
    {
        print_usage(argv[0]);
        return 1;
    }

    Method method = Method::PointToPoint;
    double search_radius = 0.3;
    int number_of_runs = 20;
    double start_offset_m = 0.05;
    double start_offset_deg = 0.5;
    int node_step = 1;
    double decimation = 0.0;
    double tls_decimation = 0.0;
    for (int i = 4; i < argc; i++)
    {
        const std::string option = argv[i];
        if (i + 1 >= argc)
        {
            std::cerr << "Missing value for option: " << option << std::endl;
            print_usage(argv[0]);
            return 1;
        }
        const std::string value = argv[++i];
        try
        {
            if (option == "--method")
            {
                if (value == "point_to_point")
                    method = Method::PointToPoint;
                else if (value == "point_to_plane")
                    method = Method::PointToPlane;
                else if (value == "monte_carlo")
                    method = Method::MonteCarlo;
                else
                    throw std::invalid_argument(value);
            }
            else if (option == "--search-radius")
                search_radius = std::stod(value);
            else if (option == "--runs")
                number_of_runs = std::stoi(value);
            else if (option == "--start-offset-m")
                start_offset_m = std::stod(value);
            else if (option == "--start-offset-deg")
                start_offset_deg = std::stod(value);
            else if (option == "--node-step")
                node_step = std::stoi(value);
            else if (option == "--decimation")
                decimation = std::stod(value);
            else if (option == "--tls-decimation")
                tls_decimation = std::stod(value);
            else
            {
                std::cerr << "Unknown option: " << option << std::endl;
                print_usage(argv[0]);
                return 1;
            }
        } catch (const std::exception&)
        {
            std::cerr << "Invalid value for " << option << ": " << value << std::endl;
            return 1;
        }
    }
    if (search_radius <= 0.0 || number_of_runs < 2 || start_offset_m < 0.0 || start_offset_deg < 0.0 || node_step < 1 || decimation < 0.0 ||
        tls_decimation < 0.0)
    {
        std::cerr << "Invalid options: need search radius > 0, runs >= 2, start offsets >= 0, node step >= 1 and decimations >= 0"
                  << std::endl;
        return 1;
    }

    const std::filesystem::path tlsFile{ argv[1] };
    std::string extension = tlsFile.extension().string();
    std::transform(
        extension.begin(),
        extension.end(),
        extension.begin(),
        [](unsigned char character)
        {
            return static_cast<char>(std::tolower(character));
        });

    if (!std::filesystem::is_regular_file(tlsFile))
    {
        std::cerr << "TLS file does not exist or is not a regular file: " << tlsFile.string() << std::endl;
        return 1;
    }

    if (extension != ".las" && extension != ".laz")
    {
        std::cerr << "Unsupported TLS file format: " << tlsFile.string() << " (expected .las or .laz)" << std::endl;
        return 1;
    }

    const std::vector<mandeye::Point> tlsPoints = mandeye::load(tlsFile.string());
    if (tlsPoints.empty())
    {
        std::cerr << "Failed to load TLS point cloud: " << tlsFile.string() << std::endl;
        return 1;
    }
    std::cout << "Loaded " << tlsPoints.size() << " points from " << tlsFile.string() << std::endl;

    const std::filesystem::path sessionDataFile{ argv[2] };

    if (!std::filesystem::is_regular_file(sessionDataFile))
    {
        std::cerr << "Session file does not exist or is not a regular file: " << sessionDataFile.string() << std::endl;
        return 1;
    }

    Session session;
    if (!session.load(sessionDataFile.string(), decimation > 0.0, decimation, decimation, decimation, /*calculate_offset=*/false))
    {
        std::cerr << "Failed to load session: " << sessionDataFile.string() << std::endl;
        return 1;
    }

    std::size_t sessionPointCount = 0;
    for (const PointCloud& pointCloud : session.point_clouds_container.point_clouds)
    {
        sessionPointCount += pointCloud.points_local.size();
    }
    std::cout << "Loaded session " << sessionDataFile.string() << " with " << session.point_clouds_container.point_clouds.size()
              << " chunks and " << sessionPointCount << " points" << std::endl;

    const std::filesystem::path outputFile{ argv[3] };
    std::cout << "Output will be saved to: " << outputFile.string() << std::endl;

    /////////////////////////////////////////////////////////////////////////////////////////
    std::vector<Eigen::Vector3d> target_global;

    for (const auto& p : tlsPoints)
    {
        target_global.emplace_back(p.point);
    }
    if (tls_decimation > 0.0)
    {
        PointCloud tls;
        tls.points_local = std::move(target_global);
        tls.decimate(tls_decimation, tls_decimation, tls_decimation);
        target_global = std::move(tls.points_local);
        std::cout << "TLS decimated to " << target_global.size() << " points (voxel " << tls_decimation << " m)" << std::endl;
    }

    std::vector<TaitBryanPose> icp_uncertainties;
    std::vector<IcpResidualStatistics> icp_residuals; // point_to_point, point_to_plane
    std::vector<IcpSpread> icp_spreads; // monte_carlo
    std::vector<std::pair<double, double>> node_timestamps; // <lidar_timestamp, unix_timestamp>
    std::vector<Eigen::Vector3d> node_positions;

    const auto& point_clouds = session.point_clouds_container.point_clouds;
    size_t number_of_nodes = 0;
    for (const auto& pointCloud : point_clouds)
    {
        number_of_nodes += (pointCloud.local_trajectory.size() + node_step - 1) / node_step;
    }
    size_t nodes_done = 0;
    const char* const method_names[] = { "point_to_point", "point_to_plane", "monte_carlo" };
    std::cout << "Computing " << method_names[static_cast<int>(method)] << " uncertainty for " << number_of_nodes << " trajectory nodes in "
              << point_clouds.size() << " chunks (search radius " << search_radius << " m";
    if (method == Method::MonteCarlo)
    {
        std::cout << ", " << number_of_runs << " ICP runs per node";
    }
    std::cout << ")" << std::endl;
    const auto start_time = std::chrono::steady_clock::now();

    for (size_t chunk_index = 0; chunk_index < point_clouds.size(); chunk_index++)
    {
        const PointCloud& pointCloud = point_clouds[chunk_index];
        std::vector<Eigen::Vector3d> source_local;
        for (const auto& p : pointCloud.points_local)
        {
            source_local.emplace_back(p);
        }

        Eigen::Affine3d m_pose = pointCloud.m_pose;

        std::vector<Eigen::Vector3d> source_global;

        for (const auto& p : source_local)
        {
            source_global.emplace_back(m_pose * p);
        }

        //
        /*std::ofstream csv("source.txt");

        for (size_t i = 0; i < source_global.size(); i++)
        {
            csv <<source_global[i].x() << ',' << source_global[i].y() << ',' << source_global[i].z() << '\n';
        }

        std::ofstream trg("target.txt");

        for (size_t i = 0; i < target_global.size(); i++)
        {
            trg <<target_global[i].x() << ',' << target_global[i].y() << ',' << target_global[i].z() << '\n';
        }*/
        //

        // Only TLS points near the chunk can be ICP neighbours: crop the target to the chunk's bounding box (points and
        // trajectory) grown by the search radius, so each ICP indexes a small cloud instead of the whole TLS scan.
        // Monte Carlo also moves the start pose by up to the start offsets (rotation about the node, at most the box
        // diagonal away), plus 1 m for ICP drift.
        Eigen::AlignedBox3d chunk_box;
        for (const auto& p : source_global)
        {
            chunk_box.extend(p);
        }
        for (const auto& n : pointCloud.local_trajectory)
        {
            chunk_box.extend((m_pose * n.m_pose).translation());
        }
        double margin = search_radius;
        if (method == Method::MonteCarlo)
        {
            const double start_offset_rad = deg2rad(start_offset_deg);
            margin += std::sqrt(3.0) * (start_offset_m + start_offset_rad * chunk_box.diagonal().norm()) + 1.0;
        }
        const Eigen::AlignedBox3d crop_box(
            chunk_box.min() - Eigen::Vector3d::Constant(margin), chunk_box.max() + Eigen::Vector3d::Constant(margin));
        std::vector<Eigen::Vector3d> target_chunk;
        for (const auto& p : target_global)
        {
            if (crop_box.contains(p))
            {
                target_chunk.emplace_back(p);
            }
        }
        std::cout << "chunk: " << source_global.size() << " points, " << target_chunk.size() << " of " << target_global.size()
                  << " TLS points within " << std::fixed << std::setprecision(2) << margin << " m of it" << std::defaultfloat
                  << std::setprecision(6) << std::endl;

        for (size_t node_index = 0; node_index < pointCloud.local_trajectory.size(); node_index += node_step)
        {
            const auto& n = pointCloud.local_trajectory[node_index];
            // "[  12/7637   0.2%] chunk 1/61 node 12/125"
            std::ostringstream where;
            where << '[' << std::setw(static_cast<int>(std::to_string(number_of_nodes).size())) << nodes_done + 1 << '/' << number_of_nodes
                  << ' ' << std::fixed << std::setprecision(1) << std::setw(5) << 100.0 * (nodes_done + 1) / number_of_nodes << "%] chunk "
                  << chunk_index + 1 << '/' << point_clouds.size() << " node " << node_index + 1 << '/'
                  << pointCloud.local_trajectory.size();
            Eigen::Affine3d m_global = m_pose * n.m_pose;

            Eigen::Affine3d m_local = m_global.inverse();

            std::vector<Eigen::Vector3d> source_local_for_single_trajectory_node;

            for (const auto& p : source_global)
            {
                source_local_for_single_trajectory_node.emplace_back(m_local * p);
            }

            PairWiseICP icp;
            icp.verbose = false;
            TaitBryanPose icp_uncertainty;
            if (method == Method::MonteCarlo)
            {
                const IcpSpread spread = icp.compute_icp_montecarlo(
                    source_local_for_single_trajectory_node,
                    target_chunk,
                    m_global,
                    search_radius,
                    30,
                    number_of_runs,
                    start_offset_m,
                    deg2rad(start_offset_deg),
                    [&](int run)
                    {
                        std::cout << '\r' << where.str() << ": ICP run " << run << '/' << number_of_runs << std::flush;
                    });
                std::cout << '\r';
                const Eigen::Matrix<double, 6, 1> sigma = spread.covariance.diagonal().cwiseSqrt();
                icp_uncertainty = TaitBryanPose{ sigma(0), sigma(1), sigma(2), sigma(3), sigma(4), sigma(5) };
                icp_spreads.emplace_back(spread);
            }
            else
            {
                IcpResidualStatistics residuals;
                std::tie(icp_uncertainty, residuals) = method == Method::PointToPlane
                    ? icp.compute_icp_uncertainty_point_to_plane(
                          source_local_for_single_trajectory_node, target_chunk, m_global, search_radius)
                    : icp.compute_icp_uncertainty(source_local_for_single_trajectory_node, target_chunk, m_global, search_radius);
                icp_residuals.emplace_back(residuals);
            }
            nodes_done++;
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            const double per_node = elapsed / nodes_done;
            std::cout << where.str() << std::fixed << std::setprecision(4) << " | sigma [mm] " << icp_uncertainty.px * 1000.0 << ' '
                      << icp_uncertainty.py * 1000.0 << ' ' << icp_uncertainty.pz * 1000.0 << " [deg] " << std::setprecision(4)
                      << rad2deg(icp_uncertainty.om) << ' ' << rad2deg(icp_uncertainty.fi) << ' ' << rad2deg(icp_uncertainty.ka)
                      << std::setprecision(1) << " | " << per_node << " s/node, elapsed " << format_duration(elapsed) << ", ETA "
                      << format_duration(per_node * (number_of_nodes - nodes_done)) << std::defaultfloat << std::setprecision(6)
                      << std::endl;

            // TaitBryanPose icp_uncertainty_alternative_approach =

            icp_uncertainties.emplace_back(icp_uncertainty);
            node_timestamps.emplace_back(n.timestamps);
            node_positions.emplace_back(m_global.translation());
        }
    }

    std::ofstream csv(outputFile);
    if (!csv)
    {
        std::cerr << "Failed to open output file: " << outputFile.string() << std::endl;
        return 1;
    }

    // Standard deviations: sigma_px/py/pz in metres, sigma_om/fi/ka in radians (inf = pose not constrained).
    // point_to_point, point_to_plane: residuals, sum_squared_residuals in m^2,
    // sigma0 = sqrt(sum_squared_residuals / (observations - 6)) in metres.
    // monte_carlo: runs, converged runs and the mean converged pose minus the node pose (m, rad).
    // Then the upper triangle of the 6x6 pose covariance (px, py, pz [m], om, fi, ka [rad]), row by row; nan = not computed.
    const char* const parameter_names[] = { "px", "py", "pz", "om", "fi", "ka" };
    csv << "timestamp_lidar,timestamp_unix,x,y,z,sigma_px,sigma_py,sigma_pz,sigma_om,sigma_fi,sigma_ka,";
    if (method == Method::MonteCarlo)
    {
        csv << "number_of_runs,number_of_converged_runs,mean_offset_px,mean_offset_py,mean_offset_pz,mean_offset_om,mean_offset_fi,"
               "mean_offset_ka";
    }
    else
    {
        csv << "sum_squared_residuals,number_of_correspondences,number_of_observations,sigma0(RMS)";
    }
    for (int row = 0; row < 6; row++)
    {
        for (int col = row; col < 6; col++)
        {
            csv << ",cov_" << parameter_names[row] << '_' << parameter_names[col];
        }
    }
    csv << '\n';
    csv << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (size_t i = 0; i < icp_uncertainties.size(); i++)
    {
        const TaitBryanPose& u = icp_uncertainties[i];
        csv << node_timestamps[i].first << ',' << node_timestamps[i].second << ',' << node_positions[i].x() << ',' << node_positions[i].y()
            << ',' << node_positions[i].z() << ',' << u.px << ',' << u.py << ',' << u.pz << ',' << u.om << ',' << u.fi << ',' << u.ka;
        Eigen::Matrix<double, 6, 6> covariance;
        if (method == Method::MonteCarlo)
        {
            const IcpSpread& spread = icp_spreads[i];
            csv << ',' << spread.number_of_runs << ',' << spread.number_of_converged_runs;
            for (int k = 0; k < 6; k++)
            {
                csv << ',' << spread.mean_offset(k);
            }
            covariance = spread.covariance;
        }
        else
        {
            const IcpResidualStatistics& r = icp_residuals[i];
            csv << ',' << r.sum_squared_residuals << ',' << r.number_of_correspondences << ',' << r.number_of_observations << ','
                << r.sigma0;
            covariance = r.covariance;
        }
        for (int row = 0; row < 6; row++)
        {
            for (int col = row; col < 6; col++)
            {
                csv << ',' << covariance(row, col);
            }
        }
        csv << '\n';
    }

    if (!csv)
    {
        std::cerr << "Failed to write output file: " << outputFile.string() << std::endl;
        return 1;
    }
    std::cout << "Saved " << icp_uncertainties.size() << " node uncertainties to " << outputFile.string() << std::endl;

    return 0;
}