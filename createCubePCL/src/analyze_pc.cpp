#include <createCubePCL/analyze_pc.h>

#define GT_CUBE "cube2"
#define QD_CUBE "cube1"

AnalyzePC::AnalyzePC(){
    gt_cloud_sub = nh.subscribe("/"+ GT_CUBE + "/cloud", 1, &AnalyzePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe("/"+ QD_CUBE + "/cloud", 1, &AnalyzePC::qdCloudCb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/cloud_vis_marker",1, this);
}

void AnalyzePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, gt_cloud);
}

void AnalyzePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, qd_cloud);
}

void AnalyzePC::visualizeError(){
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";
    marker.header.stamp = ros::Time();
    marker.ns = "cube_error";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.001;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.points.clear();
    if (gt_cloud.width == qd_cloud.width && gt_cloud.height == qd_cloud.height){
        for (size_t i=0; i<gt_cloud.width; ++i){
            for (size_t j=0; j<gt_cloud.height; ++j){
                geometry_msgs::Point p;
                p.x = gt_cloud(i,j).x;
                p.y = gt_cloud(i,j).y;
                p.z = gt_cloud(i,j).z;
                marker.points.push_back(p);
                p.x = qd_cloud(i,j).x;
                p.y = qd_cloud(i,j).y;
                p.z = qd_cloud(i,j).z;
                marker.points.push_back(p);
            }
        }
    }
    else{
        ROS_ERROR("PointClouds not registered yet");
    }
    vis_pub.publish(marker);
}

void AnalyzePC::spin(){
    ros::Rate loop_rate(10);
    while(ros::ok()){
        visualizeError();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "analyze_pc");
    AnalyzePC test;
    test.spin();
    return 0;
}
