#include <gnss.h>
#include <wgs84_do_puwg92.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <GL/freeglut.h>

inline void split(std::string &str, char delim, std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;

    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}

bool GNSS::load(const std::vector<std::string> &input_file_names)
{
    // 54651848940 5156.43798828125 2009.1610107421875 122.1999969482421875 1.25 8 35.799999237060546875 nan 14:51:42 1
    // timestamp lat lon alt hdop satelites_tracked height age time fix_quality

    gnss_poses.clear();

    std::cout << "loading GNSS data from following files:" << std::endl;
    for (const auto &fn : input_file_names)
    {
        std::cout << fn << std::endl;
    }

    for (const auto &fn : input_file_names)
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
            std::vector<std::string> strs;
            split(s, ' ', strs);

            if (strs.size() == 10)
            {
                GlobalPose gp;
                std::istringstream(strs[0]) >> gp.timestamp;
                std::istringstream(strs[1]) >> gp.lat;
                std::istringstream(strs[2]) >> gp.lon;
                std::istringstream(strs[3]) >> gp.alt;
                std::istringstream(strs[4]) >> gp.hdop;
                std::istringstream(strs[5]) >> gp.satelites_tracked;
                std::istringstream(strs[6]) >> gp.height;
                std::istringstream(strs[7]) >> gp.age;
                std::istringstream(strs[8]) >> gp.time;
                std::istringstream(strs[9]) >> gp.fix_quality;

                double L = gp.lon;
                double B = gp.lat;

                wgs84_do_puwg92(B, L, &gp.y, &gp.x);
                // std::cout << std::setprecision(20) << B << " " << L << " " << gp.x << " " << gp.y << std::endl;
                if (Eigen::Vector3d(gp.y, gp.x, gp.alt).norm() > 0)
                {
                    gnss_poses.push_back(gp);
                }
            }
        }
        infile.close();
    }

    std::sort(gnss_poses.begin(), gnss_poses.end(), [](GNSS::GlobalPose &a, GNSS::GlobalPose &b)
              { return (a.timestamp < b.timestamp); });

    offset_x = 0;
    offset_y = 0;
    offset_alt = 0;
    for (int i = 0; i < gnss_poses.size(); i++)
    {
        offset_x += gnss_poses[i].x;
        offset_y += gnss_poses[i].y;
        offset_alt += gnss_poses[i].alt;
    }
    offset_x /= gnss_poses.size();
    offset_y /= gnss_poses.size();
    offset_alt /= gnss_poses.size();

    return true;
}

void GNSS::render(const PointClouds &point_clouds_container)
{
    glColor3f(1, 1, 1);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < gnss_poses.size(); i++)
    {
        glVertex3f(gnss_poses[i].x - offset_x, gnss_poses[i].y - offset_y, gnss_poses[i].alt - offset_alt);
    }
    glEnd();

    if (show_correspondences)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        for (const auto &pc : point_clouds_container.point_clouds)
        {
            for (int i = 0; i < gnss_poses.size(); i++)
            {
                double time_stamp = gnss_poses[i].timestamp;

                auto it = std::lower_bound(pc.local_trajectory.begin(), pc.local_trajectory.end(),
                                           time_stamp, [](const PointCloud::LocalTrajectoryNode &lhs, const double &time) -> bool
                                           { return lhs.timestamp < time; });

                int index = it - pc.local_trajectory.begin();

                if (index > 0 && index < pc.local_trajectory.size())
                {

                    if (fabs(time_stamp - pc.local_trajectory[index].timestamp) < 10e12)
                    {
                        auto m = pc.m_pose * pc.local_trajectory[index].m_pose;
                        glVertex3f(m(0, 3), m(1, 3), m(2, 3));
                        glVertex3f(gnss_poses[i].x - offset_x, gnss_poses[i].y - offset_y, gnss_poses[i].alt - offset_alt);
                    }
                }
            }
        }
        glEnd();
    }
}
