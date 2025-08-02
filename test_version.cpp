#include "lidar_odometry_utils.h"
#include <iostream>

int main() {
    std::cout << "Testing version system..." << std::endl;
    std::cout << "Software version: " << get_software_version() << std::endl;
    return 0;
}
