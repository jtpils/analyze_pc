/* File : pcl_cube.cpp
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube PointCloud creation. Function definitions of the class
 */

#include <createCubePCL/pcl_cube.h>

PCLCube::PCLCube(){
    cube_center.x = 0;
    cube_center.y = 0;
    cube_center.z = 0;
    cube_axes[0] = pcl::Normal(0,0,1); //The Z-axis
    cube_axes[1] = pcl::Normal(0,1,0); //The Y-axis
    std::cerr << cube_axes[0] << cube_axes[1] << "\n";

    dense = false;
    scale = 1.0;
};

void PCLCube::savetoFile(){
    savetoFile("cube_pcl.pcd");
}

void PCLCube::savetoFile(std::string fn){
  ROS_INFO("Saved %d data points to %s\n",(int)cube_cloud.points.size(),fn.c_str());
  pcl::io::savePCDFileASCII (fn, cube_cloud);
}

void PCLCube::generatePlanePoints(pcl::PointNormal center, float scale){
    //std::cerr << cube_cloud.width << " " << cube_cloud.height << "\n";
    for (size_t i=0; i<cube_cloud.width/6; ++i){
        for(size_t j=0; j<cube_cloud.height; ++j){
            cube_cloud(i,j) = pcl::PointXYZ(0.0,0.0,0.0);
            //TODO : get correct coordinates for points
            //
        }
    }
}

void PCLCube::findFaceCenters(pcl::PointNormal &fc, pcl::Normal normal, bool direction){

}

void PCLCube::generatePoints(){
    float dense_scale = scale;
    pcl::PointNormal face_centers[6];
    bool direction = true;
    if (dense){
        dense_scale*=DENSE_FACTOR;
    }
    cube_cloud.header.frame_id = "cube_cloud";
    cube_cloud.height = POINTS_PER_UNIT*dense_scale;
    cube_cloud.width = 6*POINTS_PER_UNIT*dense_scale;
    cube_cloud.points.resize(cube_cloud.width*cube_cloud.height);
    //std::cerr << cube_cloud.width << " " << cube_cloud.height << "\n";
    //std::cerr << POINTS_PER_UNIT << " " << DENSE_FACTOR << " " << dense_scale << "\n";
    for (int i=0; i<6; ++i){
        // TODO : get the correct face_centers with normals
        findFaceCenters(face_centers[i],cube_axes[i/2],direction);
        direction = (!direction);
        generatePlanePoints(face_centers[i], scale);
    }
}

void PCLCube::spin(){
    ros::spin();
}
