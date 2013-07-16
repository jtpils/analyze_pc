#include <createCubePCL/pcl_cube.h>

int main (int argc, char ** argv){
    ros::init (argc, argv, "two_cubes_compare");
    PCLCube* c1 = new PCLCube("cube1");
    PCLCube* c2 = new PCLCube("cube2");
    c1->colorIt(255,0,0);
    c2->colorIt(0,255,0);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        c1->publishPointCloud();
        c2->publishPointCloud();
        ros::spinOnce(); //Not getting any callbacks but just in case :P
        loop_rate.sleep();
    }
    return 0;
}
