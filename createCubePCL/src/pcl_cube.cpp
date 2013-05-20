#include <createCubePCL/pcl_cube.h>

PCLCube::PCLCube(){
    cube_center.x = 0;
    cube_center.y = 0;
    cube_center.z = 0;

    dense = true;
    generatePoints();
}

void PCLCube::savetoFile(){
    savetoFile("cube_pcl.pcd");
}

void PCLCube::savetoFile(std::string fn){
  pcl::io::savePCDFileASCII (fn, cube_cloud);
  ROS_INFO("Saved %d data points to %s\n",(int)cube_cloud.points.size(),fn.c_str());

}

void PCLCube::generatePoints(){

}
