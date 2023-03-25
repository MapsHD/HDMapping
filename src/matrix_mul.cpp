#include <iostream>
#include <fstream>
#include <Eigen/Eigen>

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cout << "USAGE: " << argv[0] << "file_with_4x4_matrix file_with_4x4_matrix" << std::endl;
    }
    else
    {
        Eigen::Affine3d m1 = Eigen::Affine3d::Identity();
        Eigen::Affine3d m2 = Eigen::Affine3d::Identity();
        {
            std::ifstream infile(argv[1]);
            if (!infile.good())
            {
                std::cout << "problem with file: '" << argv[1] << "'" << std::endl;
                return false;
            }
            std::string line;
            std::getline(infile, line);
            std::istringstream iss1(line);
            iss1 >> m1(0, 0) >> m1(0, 1) >> m1(0, 2) >> m1(0, 3);

            std::getline(infile, line);
            std::istringstream iss2(line);
            iss2 >> m1(1, 0) >> m1(1, 1) >> m1(1, 2) >> m1(1, 3);

            std::getline(infile, line);
            std::istringstream iss3(line);
            iss3 >> m1(2, 0) >> m1(2, 1) >> m1(2, 2) >> m1(2, 3);

            infile.close();
        }
        {
            std::ifstream infile(argv[2]);
            if (!infile.good())
            {
                std::cout << "problem with file: '" << argv[2] << "'" << std::endl;
                return false;
            }
            std::string line;
            std::getline(infile, line);
            std::istringstream iss1(line);
            iss1 >> m2(0, 0) >> m2(0, 1) >> m2(0, 2) >> m2(0, 3);

            std::getline(infile, line);
            std::istringstream iss2(line);
            iss2 >> m2(1, 0) >> m2(1, 1) >> m2(1, 2) >> m2(1, 3);

            std::getline(infile, line);
            std::istringstream iss3(line);
            iss3 >> m2(2, 0) >> m2(2, 1) >> m2(2, 2) >> m2(2, 3);

            infile.close();
        }

        std::cout << (m1*m2).matrix() << std::endl;
    }
    return 0;
}