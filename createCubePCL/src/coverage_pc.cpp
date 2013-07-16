#include <createCubePCL/coverage_pc.h>
#include "dm_colors.hpp"

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud, pcl::KdTreeFLANN<Point>& kdtree, int K);

std::string qd_name;
std::string gt_name;

CoveragePC::CoveragePC():
gt_cloud(new pcl::PointCloud<Point>),
qd_cloud(new pcl::PointCloud<Point>),
cov_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    std::string gt_cloud_topic_name = "/"+gt_name+"/cloud";
    std::string qd_cloud_topic_name = "/"+qd_name+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &CoveragePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &CoveragePC::qdCloudCb, this);
    coverage_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/coverage_pc/"+gt_name+"_"+qd_name+"_coverage",1);
}

void CoveragePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *gt_cloud);
}

void CoveragePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *qd_cloud);
}

void CoveragePC::findCorrespondences(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(gt_cloud);
    for (size_t i=0; i<qd_cloud->width; ++i){
        Point searchPoint = qd_cloud->points[i];
        int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree, 1)[0];
        Point resultPoint = gt_cloud->points[point_index];
    }

}

void CoveragePC::spin(){
    ros::Rate loop_rate(10);
    while (ros::ok()){
        ros::spinOnce();
        findCorrespondences();
        loop_rate.sleep();
    }

}

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud,
        pcl::KdTreeFLANN<Point>& kdtree, int K){
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        return pointIdxNKNSearch;
    }
    pointIdxNKNSearch.clear();
    return pointIdxNKNSearch;
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "coverage_pc");
    gt_name = "cube1";
    qd_name = "cube2";
    if (argc == 3){
        gt_name = argv[1];
        qd_name = argv[2];
    }
    CoveragePC test;
    test.spin();
    return 0;
}
