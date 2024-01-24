#include "BeanBox2.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " intrinsics depth_path color_leaf_path date_outpath" << std::endl;
        return 1;
    }

    BeanBox2 test;
    std::string intrinsics = argv[1];
    std::string depth_path = argv[2];
    std::string color_leaf_path = argv[3];
    std::string date_outpath = argv[4];
    
    test.run(intrinsics, depth_path, color_leaf_path, date_outpath);

    return 0;
}
