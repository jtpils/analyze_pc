#include <createCubePCL/coverage_pc.h>
#include "dm_colors.hpp"

#define WORLD_FRAME "/world"

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud, pcl::KdTreeFLANN<Point>& kdtree, int K);
void convertPoints(pcl::PointXYZRGB& p, Point& q);
void colorIt(pcl::PointXYZRGB& p, int color);

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
    max_correspondence_distance = 0.05*0.05;
    nh.setParam("/coverage_pc/max_correspondence_distance",
            max_correspondence_distance);
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
    ROS_INFO("Find correspondences between pointclouds");
    pcl::KdTreeFLANN<Point> kdtree_gt;
    pcl::KdTreeFLANN<Point> kdtree_qd;
    kdtree_gt.setInputCloud(gt_cloud);
    kdtree_qd.setInputCloud(qd_cloud);
    std::vector<bool> gt_covered;
    std::vector<bool> qd_covered;
    gt_covered.clear();
    qd_covered.clear();
    cov_cloud->points.clear();
    float distance;
    for (size_t i=0; i<gt_cloud->size(); ++i){
        gt_covered.push_back(false);
    }
    for (size_t i=0; i<qd_cloud->width; ++i){
        pcl::PointXYZRGB point;
        Point searchPoint = qd_cloud->points[i];
        int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree_gt, 1)[0];
        Point resultPoint = gt_cloud->points[point_index];
        Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
        Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
        distance = (p1-p2).squaredNorm();
        if (distance < max_correspondence_distance){
            gt_covered[point_index] = true;
            convertPoints(point, resultPoint);
            colorIt(point, 0);
            cov_cloud->points.push_back(point);
            qd_covered.push_back(true);
            convertPoints(point, searchPoint);
            colorIt(point, 0);
            cov_cloud->points.push_back(point);
        }else{
            qd_covered.push_back(false);
            convertPoints(point, searchPoint);
            colorIt(point, 1);
            cov_cloud->points.push_back(point);
        }
    }
    for (size_t i=0; i<gt_cloud->size(); ++i){
        pcl::PointXYZRGB point;
        if (!gt_covered[i]){
            Point searchPoint = gt_cloud->points[i];
            int rev_index = findNearestPointIndices(searchPoint, qd_cloud, kdtree_qd, 1)[0];
            Point resultPoint = qd_cloud->points[rev_index];
            Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
            Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
            distance = (p1-p2).squaredNorm();
            if (distance < max_correspondence_distance){
                gt_covered[i] = true;
                convertPoints(point, searchPoint);
                colorIt(point, 2);
                cov_cloud->points.push_back(point);
            }
        }
    }

    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(*cov_cloud, pc);
    pc.header.frame_id= WORLD_FRAME;
    coverage_cloud_pub.publish(pc);
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

void convertPoints(pcl::PointXYZRGB& p, Point& q){
    p.x = q.x;
    p.y = q.y;
    p.z = q.z;
}

void colorIt(pcl::PointXYZRGB& p, int color){
    p.r = 0;
    p.g = 0;
    p.b = 0;
    switch(color){
        case 0: p.r = 1;
        case 1: p.g = 1;
        case 2: p.b = 1;
    }
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
