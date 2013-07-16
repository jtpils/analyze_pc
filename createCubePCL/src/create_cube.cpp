#include <createCubePCL/pcl_cube.h>

int main(int argc, char ** argv){
    ros::init (argc, argv, "pcl_cube_creation");
    PCLCube c;
    c.savetoFile();
    return 0;
}
