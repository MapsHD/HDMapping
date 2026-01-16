#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <structures.h>
#include <transformations.h>

#include <point_clouds.h>

inline void split(std::string& str, char delim, std::vector<std::string>& out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cout << "USAGE: " << argv[0] << " input_point_cloud output_point_cloud multiplier(e.g. 1000)" << std::endl;
        return 1;
    }

    std::filesystem::path trj_path = argv[1];

    if (std::filesystem::exists(trj_path))
    {
        std::cout << " loading.. ";

        std::vector<PointCloud::LocalTrajectoryNode> local_trajectory;
        //
        std::ifstream infile(trj_path.string());
        if (!infile.good())
        {
            std::cout << "problem with file: '" << trj_path.string() << "'" << std::endl;
            return false;
        }

        std::string s;
        getline(infile, s); // csv header
        while (!infile.eof())
        {
            getline(infile, s);
            std::vector<std::string> strs;
            split(s, ' ', strs);

            if (strs.size() == 13)
            {
                PointCloud::LocalTrajectoryNode ltn;
                std::istringstream(strs[0]) >> ltn.timestamps.first;
                std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
                std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
                std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
                std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
                std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
                std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
                std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
                std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
                std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
                std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
                std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
                std::istringstream(strs[12]) >> ltn.m_pose(2, 3);

                ltn.timestamps.second = 0.0;
                ltn.imu_om_fi_ka = { 0, 0, 0 };
                local_trajectory.push_back(ltn);
            }

            if (strs.size() == 14)
            {
                PointCloud::LocalTrajectoryNode ltn;
                std::istringstream(strs[0]) >> ltn.timestamps.first;
                std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
                std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
                std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
                std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
                std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
                std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
                std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
                std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
                std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
                std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
                std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
                std::istringstream(strs[12]) >> ltn.m_pose(2, 3);
                std::istringstream(strs[13]) >> ltn.timestamps.second;
                ltn.imu_om_fi_ka = { 0, 0, 0 };

                ltn.timestamps.first *= atof(argv[3]);

                local_trajectory.push_back(ltn);
            }

            if (strs.size() == 17)
            {
                PointCloud::LocalTrajectoryNode ltn;
                std::istringstream(strs[0]) >> ltn.timestamps.first;
                std::istringstream(strs[1]) >> ltn.m_pose(0, 0);
                std::istringstream(strs[2]) >> ltn.m_pose(0, 1);
                std::istringstream(strs[3]) >> ltn.m_pose(0, 2);
                std::istringstream(strs[4]) >> ltn.m_pose(0, 3);
                std::istringstream(strs[5]) >> ltn.m_pose(1, 0);
                std::istringstream(strs[6]) >> ltn.m_pose(1, 1);
                std::istringstream(strs[7]) >> ltn.m_pose(1, 2);
                std::istringstream(strs[8]) >> ltn.m_pose(1, 3);
                std::istringstream(strs[9]) >> ltn.m_pose(2, 0);
                std::istringstream(strs[10]) >> ltn.m_pose(2, 1);
                std::istringstream(strs[11]) >> ltn.m_pose(2, 2);
                std::istringstream(strs[12]) >> ltn.m_pose(2, 3);
                std::istringstream(strs[13]) >> ltn.timestamps.second;
                std::istringstream(strs[14]) >> ltn.imu_om_fi_ka.x();
                std::istringstream(strs[15]) >> ltn.imu_om_fi_ka.y();
                std::istringstream(strs[16]) >> ltn.imu_om_fi_ka.z();

                ltn.timestamps.first *= atof(argv[3]);

                local_trajectory.push_back(ltn);
            }
        }

        std::cout << local_trajectory.size() << " local nodes" << std::endl;
        infile.close();

        // save trajectory
        std::ofstream outfile;
        outfile.open(argv[2]);
        if (!outfile.good())
        {
            std::cout << "can not save file: " << argv[2] << std::endl;
            return 3;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                   "timestampUnix_nanoseconds om_rad fi_rad ka_rad"
                << std::endl;
        for (int j = 0; j < local_trajectory.size(); j++)
        {
            outfile << std::setprecision(20) << local_trajectory[j].timestamps.first << " " << std::setprecision(10)
                    << local_trajectory[j].m_pose(0, 0) << " " << local_trajectory[j].m_pose(0, 1) << " "
                    << local_trajectory[j].m_pose(0, 2) << " " << local_trajectory[j].m_pose(0, 3) << " "
                    << local_trajectory[j].m_pose(1, 0) << " " << local_trajectory[j].m_pose(1, 1) << " "
                    << local_trajectory[j].m_pose(1, 2) << " " << local_trajectory[j].m_pose(1, 3) << " "
                    << local_trajectory[j].m_pose(2, 0) << " " << local_trajectory[j].m_pose(2, 1) << " "
                    << local_trajectory[j].m_pose(2, 2) << " " << local_trajectory[j].m_pose(2, 3) << " " << std::setprecision(20)
                    << local_trajectory[j].timestamps.second << " " << local_trajectory[j].imu_om_fi_ka.x() << " "
                    << local_trajectory[j].imu_om_fi_ka.y() << " " << local_trajectory[j].imu_om_fi_ka.z() << " " << std::endl;
        }
        outfile.close();
    }
    else
    {
        std::cout << "trajectory path: '" << trj_path.string() << "' does not exist" << std::endl;
        return 2;
    }

    return 0;
}