#include<ros/ros.h>
#include<pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "utils.hpp"

#define WORLD_FRAME "/world"

std::string laser_frame;
pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ> processed_cloud;
ros::Publisher processed_cloud_pub;
tf::StampedTransform tcw;
tf::StampedTransform tcl;
int laser_range;

void laserCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, input_cloud);
}

void processCloud(){
    if (input_cloud.points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    //std::cerr << input_cloud.header.frame_id << "\n";
    processed_cloud.points.clear();
    tcw = getTransform(input_cloud.header.frame_id, WORLD_FRAME);
    tcl = getTransform(input_cloud.header.frame_id, laser_frame);
    pcl::PointXYZ origin(tcl.getOrigin().x(),tcl.getOrigin().y(),tcl.getOrigin().z());
    //std::ofstream dout;
    //dout.open("points.txt",std::ofstream::out);
    //std::cerr << origin.x << " " << origin.y << " " << origin.z << "\n";
    for (size_t i=0; i<input_cloud.points.size(); ++i){
        pcl::PointXYZ p = input_cloud.points[i];
        float distance = (p.x-origin.x)*(p.x-origin.x) + (p.y-origin.y)*(p.y-origin.y) + (p.z-origin.z)*(p.z-origin.z);
        if (p.x*p.x+p.y*p.y==0 or distance > laser_range*laser_range-1){
        }else{
            //dout << distance << "\n";
            processed_cloud.points.push_back(p);
        }
    }
    processed_cloud.width = processed_cloud.points.size();
    //std::cerr << processed_cloud.width << "\n";
    processed_cloud.height = 1;
    processed_cloud.header.frame_id = WORLD_FRAME;
    ROS_INFO("Processed cloud published with %d points", processed_cloud.width);
    //dout.close();
}

void publishCloud(){
    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(processed_cloud, pc);
    processed_cloud_pub.publish(pc);

}

int main (int argc, char ** argv){
    ros::init (argc, argv, "bridge_cloud_process");
    ros::NodeHandle nh;
    ros::Subscriber laser_cloud_sub = nh.subscribe("/cloud", 1, &laserCloudCb);
    processed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/laser/processed_cloud",1);
    laser_frame = argv[1];
    if (laser_frame == ""){
        laser_frame = "/laser_frame";
    }
    laser_range = 70;
    nh.setParam("/bridge_process/laser_range",laser_range);
    ros::Rate loop_rate(2);
    while(ros::ok()){
        ros::spinOnce();
        nh.getParam("/bridge_process/laser_range",laser_range);
        processCloud();
        publishCloud();
        //continueLoop();
        loop_rate.sleep();
    }
    return 0;
}
