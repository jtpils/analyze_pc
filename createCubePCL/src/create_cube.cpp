#include <createCubePCL/pcl_cube.h>

int main(int argc, char ** argv){
    ros::init (argc, argv, "pcl_cube_creation");
    PCLCube* c = new PCLCube;
    c->generatePoints();
    c->spin();
    //c.savetoFile();
    return 0;
}
