#include <createCubePCL/analyze_pc.h>

AnalyzePC::AnalyzePC(){
    gt_cloud_sub = nh.subscribe("/cube2/cloud", 1, &AnalyzePC::gtCloudCb, this);
    er_cloud_sub = nh.subscribe("/cube1/cloud", 1, &AnalyzePC::erCloudCb, this);
}

void AnalyzePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, gt_cloud);
}

void AnalyzePC::erCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, er_cloud);
}

void AnalyzePC::spin(){
    ros::Rate loop_rate(50);
    while(ros::ok()){
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
