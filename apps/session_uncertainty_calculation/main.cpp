#include <iostream>

int main(int argc, char* argv[])
{
    std::cout << "Session Uncertainty Calculation" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "This program calculates the uncertainty of a session for each trajectory node as ICP/Hessian and Error Propagation Law "
                 "assuming having TLS ground truth."
              << std::endl;

    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <TLS(laz format)> <session_data> <output_folder>" << std::endl;
        return 1;
    }

    return 0;
}