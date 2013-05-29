/* File : pcl_cube.cpp
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube PointCloud creation. Function definitions of the class
 */

#include <createCubePCL/pcl_cube.h>

pcl::Normal findNormal(pcl::Normal n1, pcl::Normal n2);

PCLCube::PCLCube(std::string name):
rng(static_cast<unsigned> (time(0))),
gaussian_dist(0,1)
{
    cube_name = name;
    cube_center.x = 0;
    cube_center.y = 0;
    cube_center.z = 0;
    cube_axes[0] = pcl::Normal(0,1,0); //The Y-axis
    cube_axes[1] = pcl::Normal(0,0,1); //The Z-axis
    cube_axes[2] = findNormal(cube_axes[0],cube_axes[1]);

    dense = false;
    noise = false;
    scale = 1.0;
    std::string topic_name = "/"+cube_name+"/cloud";
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name,1);
    generator = new GaussianGen(rng, gaussian_dist);

    //_spinner.= ros::AsyncSpinner(1);
    generatePoints();
};

void PCLCube::savetoFile(){
    savetoFile("cube_pcl.pcd");
}

pcl::Normal findNormal(pcl::Normal n1, pcl::Normal n2){
    pcl::Normal n3;
    Eigen::Vector3d v1(n1.normal[0], n1.normal[1], n1.normal[2]);
    Eigen::Vector3d v2(n2.normal[0], n2.normal[1], n2.normal[2]);
    Eigen::Vector3d v3 = v1.cross(v2);
    n3.normal[0] = v3[0];
    n3.normal[1] = v3[1];
    n3.normal[2] = v3[2];
    return n3;
}

void PCLCube::savetoFile(std::string fn){
  ROS_INFO("Saved %d data points to %s\n",(int)cube_cloud.points.size(),fn.c_str());
  pcl::io::savePCDFileASCII (fn, cube_cloud);
}

pcl::PointNormal  PCLCube::findFaceCenter(int index, bool direction){
    pcl::PointNormal fc;
    pcl::Normal n = cube_axes[index];
    int factor = 1;
    if (!direction){
        factor = -1;
    }
    // #TODO Make sure that normal is normalized!
    fc.x = cube_center.x + factor*n.normal[0]*scale/2;
    fc.y = cube_center.y + factor*n.normal[1]*scale/2;
    fc.z = cube_center.z + factor*n.normal[2]*scale/2;
    fc.normal[0] = factor*n.normal[0];
    fc.normal[1] = factor*n.normal[1];
    fc.normal[2] = factor*n.normal[2];
    return fc;
}

void PCLCube::generatePlanePoints(pcl::PointNormal center, int index){
    pcl::Normal n1 = cube_axes[(index/2+1)%3];
    pcl::Normal n2 = cube_axes[(index/2+2)%3];
    pcl::Normal n3 = cube_axes[(index/2+3)%3];
    size_t width = cube_cloud.width/6;
    size_t height = cube_cloud.height;
    float error_n1=0, error_n2=0, error_n3=0;
    for (size_t i=index*width; i<(index+1)*width; ++i){
        float n1_factor = scale*(1.0*i/width-index-0.5);
        for(size_t j=0; j<height; ++j){
            float n2_factor = scale*(1.0*j/width-0.5);
            pcl::PointXYZRGB point;
            if (noise){
                float normal_sigma = NORMAL_SIGMA_FACTOR*scale;
                float inplane_sigma = INPLANE_SIGMA_FACTOR*scale;
                if (dense){
                    inplane_sigma = inplane_sigma/DENSE_FACTOR;
                }
                error_n1 = getGaussian(inplane_sigma);
                error_n2 = getGaussian(inplane_sigma);
                error_n3 = getGaussian(normal_sigma);
            }
            // #TODO Make sure that normal is normalized!
            point.x = center.x + (n1_factor+error_n1)*n1.normal[0] + (n2_factor+error_n2)*n2.normal[0] + error_n3*n3.normal[0];
            point.y = center.y + (n1_factor+error_n1)*n1.normal[1] + (n2_factor+error_n2)*n2.normal[1] + error_n3*n3.normal[1];
            point.z = center.z + (n1_factor+error_n1)*n1.normal[2] + (n2_factor+error_n2)*n2.normal[2]+error_n3*n3.normal[2];
            cube_cloud(i,j) = point;
        }
    }
}

void PCLCube::generatePoints(){
    float dense_scale = scale;
    bool direction = true;
    if (dense){
        dense_scale*=DENSE_FACTOR;
    }
    cube_cloud.header.frame_id = "laser_"+cube_name;
    cube_cloud.height = POINTS_PER_UNIT*dense_scale;
    cube_cloud.width = 6*POINTS_PER_UNIT*dense_scale;
    cube_cloud.points.resize(cube_cloud.width*cube_cloud.height);
    for (int i=0; i<6; ++i){
        generatePlanePoints(findFaceCenter(i/2, direction), i);
        direction = (!direction);
    }
}

double PCLCube::getGaussian(double variance){
    return getGaussian(0, variance);
}

double PCLCube::getGaussian(double mean, double variance){
    return mean + (*generator)()*sqrt(variance);
}

void PCLCube::addNoise(){
    noise = true;
    generatePoints();
}

void PCLCube::addNoise(GaussianGen& gen){
    noise = true;
    // Adds noise to all points
    generator = new GaussianGen(gen);
    generatePoints();
}

void PCLCube::addNoiseToCenter(){
    pcl::PointXYZ new_center;
    float center_sigma = CENTER_SIGMA_FACTOR*scale;
    new_center.x = cube_center.x + getGaussian(center_sigma);
    new_center.y = cube_center.y + getGaussian(center_sigma);
    new_center.z = cube_center.z + getGaussian(center_sigma);
    changeCenterTo(new_center);
}

void PCLCube::addNoiseToCenter(GaussianGen& gen){
    generator = new GaussianGen(gen);
    addNoiseToCenter();
}

void PCLCube::addNoiseToOrientation(){
    for(int i=0; i<3; ++i){
        Eigen::Vector3f axis;
    }
    //generatePoints();
}

void PCLCube::addNoiseToOrientation(GaussianGen& gen){
    generator = new GaussianGen(gen);
    addNoiseToOrientation();
}

void PCLCube::changeCenterTo(pcl::PointXYZ new_center){
    pcl::PointXYZ diff_centers;
    diff_centers.x = new_center.x - cube_center.x;
    diff_centers.y = new_center.y - cube_center.y;
    diff_centers.z = new_center.z - cube_center.z;
    for (size_t i=0; i<cube_cloud.width; ++i){
        for (size_t j=0; j<cube_cloud.height; ++j){
            cube_cloud(i,j).x = cube_cloud(i,j).x + diff_centers.x;
            cube_cloud(i,j).y = cube_cloud(i,j).y + diff_centers.y;
            cube_cloud(i,j).z = cube_cloud(i,j).z + diff_centers.z;
        }
    }
    cube_center = new_center;
}

void PCLCube::colorIt(uint8_t r, uint8_t g, uint8_t b){
    for (size_t i=0; i<cube_cloud.width; ++i){
        for (size_t j=0; j<cube_cloud.height; ++j){
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cube_cloud(i,j).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
}

void PCLCube::publishPointCloud(){
    sensor_msgs::PointCloud2 temp_msg;
    pcl::toROSMsg(cube_cloud,temp_msg);
    point_cloud_pub.publish(temp_msg);
}

void PCLCube::spin(){
    ros::Rate loop_rate(100);
    while(ros::ok()){
        publishPointCloud();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
