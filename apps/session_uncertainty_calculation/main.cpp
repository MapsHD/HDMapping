#include "Core/color_las_loader.h"
#include "Core/pair_wise_iterative_closest_point.h"
#include "Core/session.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <iostream>
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

    for (const auto& p : tlsPoints){
        target_global.emplace_back(p.point);
    }

    for(const auto& pointCloud : session.point_clouds_container.point_clouds){
        std::vector<Eigen::Vector3d> source_local;
        for(const auto& p : pointCloud.points_local){
            source_local.emplace_back(p);
        }

        Eigen::Affine3d m_pose = pointCloud.m_pose;

        PairWiseICP icp;
        icp.compute(source_local, target_global, 0.1, 1, m_pose);
    }



        // session.

        /*PairWiseICP icp;
        auto m_pose = affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);
        // std::vector<Eigen::Vector3d> source = point_clouds_container.point_clouds[edges[index_active_edge].index_to].points_local;
        // std::vector<Eigen::Vector3d> target = point_clouds_container.point_clouds[edges[index_active_edge].index_from].points_local;

        ////////////////////////////////
        std::vector<Eigen::Vector3d> source;
        auto& e = edges[index_active_edge];
        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
        {
            int index_src = e.index_to + i;
            if (index_src >= 0 && index_src < point_clouds_container.point_clouds.size())
            {
                Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
                for (int k = 0; k < point_clouds_container.point_clouds[index_src].points_local.size(); k++)
                {
                    Eigen::Vector3d p_g = m_src * point_clouds_container.point_clouds[index_src].points_local[k];
                    source.push_back(p_g);
                }
                // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
            }
        }
        std::vector<Eigen::Vector3d> target;

        for (int i = -num_edge_extended_before; i <= num_edge_extended_after; i++)
        {
            int index_trg = e.index_from + i;
            if (index_trg >= 0 && index_trg < point_clouds_container.point_clouds.size())
            {
                Eigen::Affine3d m_trg = point_clouds_container.point_clouds.at(index_trg).m_pose;
                for (int k = 0; k < point_clouds_container.point_clouds[index_trg].points_local.size(); k++)
                {
                    Eigen::Vector3d p_g = m_trg * point_clouds_container.point_clouds[index_trg].points_local[k];
                    target.push_back(p_g);
                }
                // point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
            }
        }

        Eigen::Affine3d m_src_inv = point_clouds_container.point_clouds[e.index_to].m_pose.inverse();

        for (auto& p : source)
        {
            p = m_src_inv * p;
        }

        Eigen::Affine3d m_trg_inv = point_clouds_container.point_clouds[e.index_from].m_pose.inverse();

        for (auto& p : target)
        {
            p = m_trg_inv * p;
        }
        ////////////////////////////////

        if (icp.compute(source, target, search_radius, number_of_iterations, m_pose))
        {
            edges[index_active_edge].relative_pose_tb = pose_tait_bryan_from_affine_matrix(m_pose);
            return true;
        }*/

        //////////////////////////////////////////////////////////////////////////////////////////

        return 0;
}