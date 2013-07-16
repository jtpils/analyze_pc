#include<ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "utils.hpp"

#define LASER_FRAME "/laser_frame"
#define WORLD_FRAME "/world"

pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ> processed_cloud;
ros::Publisher processed_cloud_pub;
tf::StampedTransform tcw;
tf::StampedTransform tcl;

void laserCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, input_cloud);
}

void processCloud(){
    processed_cloud.points.clear();
    processed_cloud.header.frame_id = input_cloud.header.frame_id;
    tcw = getTransform(input_cloud.header.frame_id, WORLD_FRAME);
    tcl = getTransform(input_cloud.header.frame_id, LASER_FRAME);
    for (size_t i=0; i<input_cloud.points.size(); ++i){
        pcl::PointXYZ p = input_cloud.points[i];
        transformPoint(p, tcl);
        pcl::PointXYZ origin(0,0,0);
        if (pcl::euclideanDistance(p,origin) == 70.0){

        }else{
            processed_cloud.points.push_back(input_cloud.points[i]);
        }
    }
    processed_cloud.width = processed_cloud.points.size();
    std::cerr << processed_cloud.width << "\n";
    processed_cloud.height = 1;
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
    while(ros::ok()){
        ros::spinOnce();
        processCloud();
        publishCloud();
    }
    return 0;
}
