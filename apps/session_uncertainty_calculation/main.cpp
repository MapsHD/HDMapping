#include "Core/color_las_loader.h"
#include "Core/pair_wise_iterative_closest_point.h"
#include "Core/session.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

int main(int argc, char* argv[])
{
    std::cout << "Session Uncertainty Calculation" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "This program calculates the uncertainty of a session for each trajectory node from inverse ICP/Hessian assuming having "
                 "TLS ground truth."
              << std::endl;

    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <TLS(laz format)> <session_data> <output_file>" << std::endl;
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
    if (!session.load(sessionDataFile.string(), /*is_decimate=*/false, 0.0, 0.0, 0.0, /*calculate_offset=*/false))
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

    std::vector<TaitBryanPose> icp_uncertainties;
    std::vector<IcpResidualStatistics> icp_residuals;
    std::vector<std::pair<double, double>> node_timestamps; // <lidar_timestamp, unix_timestamp>
    std::vector<Eigen::Vector3d> node_positions;

    for (const auto& pointCloud : session.point_clouds_container.point_clouds)
    {
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

        for (const auto& n : pointCloud.local_trajectory)
        {
            Eigen::Affine3d m_global = m_pose * n.m_pose;

            Eigen::Affine3d m_local = m_global.inverse();

            std::vector<Eigen::Vector3d> source_local_for_single_trajectory_node;

            for (const auto& p : source_global)
            {
                source_local_for_single_trajectory_node.emplace_back(m_local * p);
            }

            PairWiseICP icp;
            IcpResidualStatistics residuals;
            TaitBryanPose icp_uncertainty =
                icp.compute_icp_uncertainty(source_local_for_single_trajectory_node, target_global, m_global, 0.3, &residuals);
            // icp.compute_icp_uncertainty_point_to_plane(source_local_for_single_trajectory_node, target_global, m_global, 0.3);
            std::cout << "sigma_px=" << icp_uncertainty.px << ", "
                      << "sigma_py=" << icp_uncertainty.py << ", "
                      << "sigma_pz=" << icp_uncertainty.pz << ", "
                      << "sigma_om=" << icp_uncertainty.om << ", "
                      << "sigma_fi=" << icp_uncertainty.fi << ", "
                      << "sigma_ka=" << icp_uncertainty.ka << std::endl;

            // TaitBryanPose icp_uncertainty_alternative_approach =

            icp_uncertainties.emplace_back(icp_uncertainty);
            icp_residuals.emplace_back(residuals);
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
    // Residuals: sum_squared_residuals in m^2, sigma0 = sqrt(sum_squared_residuals / (observations - 6)) in metres.
    csv << "timestamp_lidar,timestamp_unix,x,y,z,sigma_px,sigma_py,sigma_pz,sigma_om,sigma_fi,sigma_ka,"
           "sum_squared_residuals,number_of_correspondences,number_of_observations,sigma0(RMS)\n";
    csv << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (size_t i = 0; i < icp_uncertainties.size(); i++)
    {
        const TaitBryanPose& u = icp_uncertainties[i];
        const IcpResidualStatistics& r = icp_residuals[i];
        csv << node_timestamps[i].first << ',' << node_timestamps[i].second << ',' << node_positions[i].x() << ',' << node_positions[i].y()
            << ',' << node_positions[i].z() << ',' << u.px << ',' << u.py << ',' << u.pz << ',' << u.om << ',' << u.fi << ',' << u.ka << ','
            << r.sum_squared_residuals << ',' << r.number_of_correspondences << ',' << r.number_of_observations << ',' << r.sigma0 << '\n';
    }

    if (!csv)
    {
        std::cerr << "Failed to write output file: " << outputFile.string() << std::endl;
        return 1;
    }
    std::cout << "Saved " << icp_uncertainties.size() << " node uncertainties to " << outputFile.string() << std::endl;

    return 0;
}