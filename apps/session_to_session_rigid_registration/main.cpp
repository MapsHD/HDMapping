#include <iostream>

int main(int argc, char *argv[]){
    std::cout << "Session to Session Rigid Registration" << std::endl;
    std::cout << "-----------------------------------" << std::endl;    
    std::cout << "This program performs rigid registration between two sessions and saves the result (rigidly transformed session_source) to a PLY file." << std::endl;

    if(argc < 4){
        std::cerr << "Usage: " << argv[0] << " <session_target> <session_source> <output_ply>" << std::endl;
        return 1;
    }

    return 0;
}