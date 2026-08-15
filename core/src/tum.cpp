#include <pch/pch.h>

#include <Core/tum.h>

#include <Eigen/Eigen>
#include <spdlog/spdlog.h>

namespace
{
    void split(const std::string& str, char delim, std::vector<std::string>& out)
    {
        size_t start;
        size_t end = 0;

        while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
        {
            end = str.find(delim, start);
            out.push_back(str.substr(start, end - start));
        }
    }
} // namespace

bool TUM::load_data_from_tum(const std::vector<std::string>& input_file_names, bool subtract_first_pose)
{
    tum_poses.clear();

    std::cout << "loading TUM trajectory data from following files:" << std::endl;
    for (const auto& fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto& fn : input_file_names)
    {
        std::ifstream infile(fn);
        if (!infile.good())
        {
            std::cout << "problem with file: '" << fn << "'" << std::endl;
            return false;
        }
        std::string s;
        while (!infile.eof())
        {
            getline(infile, s);

            if (s.empty() || s[0] == '#')
            {
                continue;
            }

            std::vector<std::string> strs;
            split(s, ' ', strs);

            if (strs.size() >= 8)
            {
                TUM::TumPose tp;
                std::istringstream(strs[0]) >> tp.timestamp;
                std::istringstream(strs[1]) >> tp.x;
                std::istringstream(strs[2]) >> tp.y;
                std::istringstream(strs[3]) >> tp.z;
                std::istringstream(strs[4]) >> tp.qx;
                std::istringstream(strs[5]) >> tp.qy;
                std::istringstream(strs[6]) >> tp.qz;
                std::istringstream(strs[7]) >> tp.qw;

                if (std::isfinite(tp.x) && std::isfinite(tp.y) && std::isfinite(tp.z) && std::isfinite(tp.qx) && std::isfinite(tp.qy) &&
                    std::isfinite(tp.qz) && std::isfinite(tp.qw))
                {
                    tum_poses.push_back(tp);
                }
            }
        }
        infile.close();
    }

    std::sort(
        tum_poses.begin(),
        tum_poses.end(),
        [](const TUM::TumPose& a, const TUM::TumPose& b)
        {
            return (a.timestamp < b.timestamp);
        });

    std::cout << "loaded " << tum_poses.size() << " TUM poses" << std::endl;

    if (subtract_first_pose && !tum_poses.empty())
    {
        const TumPose& first = tum_poses.front();
        Eigen::Affine3d T0 = Eigen::Affine3d::Identity();
        T0.translate(Eigen::Vector3d(first.x, first.y, first.z));
        T0.rotate(Eigen::Quaterniond(first.qw, first.qx, first.qy, first.qz).normalized());
        Eigen::Affine3d T0_inv = T0.inverse();

        for (auto& p : tum_poses)
        {
            Eigen::Affine3d T = Eigen::Affine3d::Identity();
            T.translate(Eigen::Vector3d(p.x, p.y, p.z));
            T.rotate(Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz).normalized());

            Eigen::Affine3d rel = T0_inv * T;
            const Eigen::Vector3d t = rel.translation();
            const Eigen::Quaterniond q(rel.rotation());

            p.x = t.x();
            p.y = t.y();
            p.z = t.z();
            p.qx = q.x();
            p.qy = q.y();
            p.qz = q.z();
            p.qw = q.w();
        }
    }

    if (!tum_poses.empty())
    {
        Eigen::Vector3d min_p(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        Eigen::Vector3d max_p(
            std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
        for (const auto& p : tum_poses)
        {
            min_p = min_p.cwiseMin(Eigen::Vector3d(p.x, p.y, p.z));
            max_p = max_p.cwiseMax(Eigen::Vector3d(p.x, p.y, p.z));
        }
        spdlog::info(
            "TUM bounding box: min=({}, {}, {}) max=({}, {}, {})", min_p.x(), min_p.y(), min_p.z(), max_p.x(), max_p.y(), max_p.z());
    }

    ++version;

    return true;
}
