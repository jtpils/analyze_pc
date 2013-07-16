/* File : pcl_cube.cpp
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube PointCloud creation. Function definitions of the class
 */

#include <createCubePCL/pcl_cube.h>

PCLCube::PCLCube(){
    cube_center.x = 0;
    cube_center.y = 0;
    cube_center.z = 0;
    cube_center.normal[0] = 0;
    cube_center.normal[1] = 0;
    cube_center.normal[2] = 1;
    //std::cerr << cube_center << "\n";

    dense = false;
}

void PCLCube::savetoFile(){
    savetoFile("cube_pcl.pcd");
}

void PCLCube::savetoFile(std::string fn){
  ROS_INFO("Saved %d data points to %s\n",(int)cube_cloud.points.size(),fn.c_str());
  pcl::io::savePCDFileASCII (fn, cube_cloud);
}

void PCLCube::generatePlanePoints(pcl::PointNormal center, float scale){
    for (size_t i=0; i<cube_cloud.width/6; ++i){
        for(size_t j=0; j<cube_cloud.height; ++j){
            std::cout << "caleed?\n";
            //cube_cloud.points(i,j) = pcl::PointXYZ(0.0,0.0,0.0);
            //TODO : get correct coordinates for points
            //
        }
    }
}

void PCLCube::generatePoints(){
    std::cerr << "Here?\n";
    cube_cloud.header.frame_id = "cube_cloud";
    float dense_scale = scale;
    if (dense){
        dense_scale*=DENSE_FACTOR;
    }
    cube_cloud.height = POINTS_PER_UNIT*dense_scale;
    cube_cloud.width = 6*POINTS_PER_UNIT*dense_scale;
    cube_cloud.points.resize(cube_cloud.width*cube_cloud.height);
    pcl::PointNormal face_centers[6];
    // TODO : get the correct face_centers with normals
    for (int i=0; i<6; ++i){
        generatePlanePoints(face_centers[i], scale);
    }
}

void PCLCube::spin(){
    ros::spin();
}
