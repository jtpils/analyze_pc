#include <createCubePCL/pcl_cube.h>
#include "utils.hpp"

int main (int argc, char ** argv){
    ros::init (argc, argv, "two_cubes_compare");
    ros::NodeHandle nh;
    RandomGen rng(static_cast<unsigned> (time(0)));
    NormalDist gaussian_dist(0,1);
    GaussianGen generator(rng,gaussian_dist);
    srand((unsigned)time(0));
    pcl::PointXYZ world_cube_center(0,0,0);
    Eigen::Quaterniond world_cube_orientation;
    world_cube_orientation.setIdentity();
    PCLCube* c1 = new PCLCube("cube1");
    PCLCube* c2 = new PCLCube("cube2");
    //c1->addNoiseToCenter(generator);
    //c1->addNoiseToOrientation(generator);
    c1->addNoise(generator);
    //float occlusion_fraction[6] = {0.5,0,0,0,0,0.5};
    float occlusion_fraction[6];
    double avg_occ=0.0;
    std::cerr << "Occlusion Fractions: ";
    for (int i=0; i<6; ++i){
        occlusion_fraction[i] = frand();
        std::cerr << occlusion_fraction[i] << " ";
        avg_occ += occlusion_fraction[i];
    }
    avg_occ = avg_occ/6;
    nh.setParam("/compare_two_cubes/avg_occ", avg_occ);
    std::cerr << "\n";
    std::cerr << "Total occluded fraction : " << avg_occ << "\n";
    //c2->setOcclusion(occlusion_fraction); // GT cube is occluded
    c1->changeCenterTo(world_cube_center, true);
    c1->changeOrientationBy(world_cube_orientation, true);
    c1->colorIt(255,0,0);
    c2->changeCenterTo(world_cube_center, true);
    c2->changeOrientationBy(world_cube_orientation);
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
