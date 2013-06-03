/* File : pcl_cube.cpp
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube PointCloud creation. Function definitions of the class
 */

#include <createCubePCL/pcl_cube.h>

pcl::Normal findNormal(pcl::Normal n1, pcl::Normal n2);
void printQuaternion(Eigen::Quaterniond);

PCLCube::PCLCube(std::string name):
rng(static_cast<unsigned> (time(0))),
gaussian_dist(0,1)
{
    cube_name = name;
    cube_center.x = 0;
    cube_center.y = 0;
    cube_center.z = 0;
    cube_orientation.setIdentity();
    cube_axes[0] = pcl::Normal(0,1,0); //The Y-axis
    cube_axes[1] = pcl::Normal(0,0,1); //The Z-axis
    cube_axes[2] = findNormal(cube_axes[0],cube_axes[1]);

    dense = false;
    noise = false;
    center_noise = false;
    scale = 1.0;
    dense_factor = 16;
    points_per_unit = 32;
    normal_sigma_factor = 2.0/points_per_unit/dense_factor;
    inplane_sigma_factor = 2.0/points_per_unit/dense_factor;
    center_sigma_factor = 0.1;
    orientation_sigma_factor = 0.01;
    std::string topic_name = "/"+cube_name+"/cloud";
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name,1);
    std::string service_name = "/"+cube_name+"/regenerate";
    std::string param_service_name = "/"+cube_name+"/set_parameters";
    regenerate_points_server = nh.advertiseService(service_name,&PCLCube::regenerateCb, this);
    get_parameters_server = nh.advertiseService(param_service_name,&PCLCube::setParamCb, this);
    generator = new GaussianGen(rng, gaussian_dist);

    //_spinner.= ros::AsyncSpinner(1);
    generatePoints();
};

void PCLCube::savetoFile(){
    savetoFile("cube_pcl.pcd");
}

void printQuaternion(Eigen::Quaterniond q){
    std::cout << "w: " << q.w() << ", x: " << q.x() << ", y: " << q.y() << ", z: " << q.z() << "\n";
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

bool PCLCube::regenerateCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    if (center_noise){
        addNoiseToCenter();
        addNoiseToOrientation();
    }
    generatePoints();
    colorIt();
    return true;
}

bool PCLCube::setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    std::string param_ns = "/"+cube_name+"/";
    nh.getParam(param_ns+"scale", scale);
    nh.getParam(param_ns+"noise", noise);
    nh.getParam(param_ns+"center_noise", center_noise);
    nh.getParam(param_ns+"dense", dense);
    nh.getParam(param_ns+"dense_factor", dense_factor);
    nh.getParam(param_ns+"points_per_unit", points_per_unit);
    nh.getParam(param_ns+"normal_sigma_factor", normal_sigma_factor);
    nh.getParam(param_ns+"inplane_sigma_factor", inplane_sigma_factor);
    nh.getParam(param_ns+"center_sigma_factor", center_sigma_factor);
    nh.getParam(param_ns+"orientation_sigma_factor", orientation_sigma_factor);
    return true;
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
                float normal_sigma = normal_sigma_factor*scale;
                float inplane_sigma = inplane_sigma_factor*scale;
                if (dense){
                    inplane_sigma = inplane_sigma/dense_factor;
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
        dense_scale*=dense_factor;
    }
    cube_cloud.header.frame_id = "laser_"+cube_name;
    cube_cloud.height = points_per_unit*dense_scale;
    cube_cloud.width = 6*points_per_unit*dense_scale;
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
    center_noise = true;
    pcl::PointXYZ new_center;
    float center_sigma = center_sigma_factor*scale;
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
    center_noise = true;
    Eigen::Quaterniond err_rot;
    double orient_sigma = orientation_sigma_factor;
    err_rot.w() = (double) 1;
    err_rot.x() = (double) getGaussian(orient_sigma);
    err_rot.y() = (double) getGaussian(orient_sigma);
    err_rot.z() = (double) getGaussian(orient_sigma);
    err_rot.normalize();
    changeOrientationBy(err_rot);
    generatePoints();
}

void PCLCube::addNoiseToOrientation(GaussianGen& gen){
    generator = new GaussianGen(gen);
    addNoiseToOrientation();
}

void PCLCube::changeCenterTo(pcl::PointXYZ new_center, bool world){
    if (world){
        tf::Vector3 diff_origin = getFrameToWorldTransform().getOrigin();
        new_center.x = new_center.x + diff_origin.x();
        new_center.y = new_center.y + diff_origin.y();
        new_center.z = new_center.z + diff_origin.z();
    }
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

void PCLCube::changeCenterBy(pcl::PointXYZ diff_center){
    pcl::PointXYZ new_center;
    new_center.x = cube_center.x + diff_center.x;
    new_center.y = cube_center.y + diff_center.y;
    new_center.x = cube_center.z + diff_center.z;
    changeCenterTo(new_center);
}

void PCLCube::changeOrientationBy(Eigen::Quaterniond rot, bool world){
    pcl::Normal axis[3];
    if (world){
        Eigen::Quaterniond tf_rot;
        tf::RotationTFToEigen(getFrameToWorldTransform().getRotation(),tf_rot);
        rot = rot*tf_rot; //or is it the other way round?
    }
    for (int i=0; i<3; ++i){
        axis[i] = cube_axes[i];
        Eigen::Vector3d new_axis = rot._transformVector(Eigen::Vector3d(axis[i].normal[0], axis[i].normal[1], axis[i].normal[2]));
        new_axis.normalize();
        cube_axes[i].normal[0] = new_axis[0];
        cube_axes[i].normal[1] = new_axis[1];
        cube_axes[i].normal[2] = new_axis[2];
    }
    generatePoints();
}

void PCLCube::colorIt(uint8_t r, uint8_t g, uint8_t b){
    for (size_t i=0; i<cube_cloud.width; ++i){
        for (size_t j=0; j<cube_cloud.height; ++j){
            rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            cube_cloud(i,j).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
}

void PCLCube::colorIt(uint32_t _rgb){
    rgb = _rgb;
}

void PCLCube::colorIt(){
    for (size_t i=0; i<cube_cloud.width; ++i){
        for (size_t j=0; j<cube_cloud.height; ++j){
            cube_cloud(i,j).rgb = *reinterpret_cast<float*>(&rgb);
        }
    }
}

void PCLCube::getTransform(){
    try {
        listener.waitForTransform("/world","laser_"+cube_name, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/world","laser_"+cube_name,ros::Time(0), world_to_frame_transform);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

tf::StampedTransform PCLCube::getFrameToWorldTransform(){
    tf::StampedTransform transform;
    getTransform();
    transform.setData(world_to_frame_transform.inverse());
    return transform;
}

tf::StampedTransform PCLCube::getWorldToFrameTransform(){
    getTransform();
    return world_to_frame_transform;
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
