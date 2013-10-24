#ifndef PCL_UTILS_HPP__
#define PCL_UTILS_HPP__
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZ Point;
float frand();
void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t);
void transformFromTo(Point& p, tf::StampedTransform t);
tf::StampedTransform getTransform(std::string from_frame, std::string to_frame);
std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud, pcl::KdTreeFLANN<Point>& kdtree, int K);
std::vector<int> findNearestPointIndices(pcl::PointXYZRGB& searchPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree, int K);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void convertPoints(pcl::PointXYZRGB& p, Point& q);
void colorIt(pcl::PointXYZRGB& p, int color);
pcl::PointXYZRGB transformPoint(pcl::PointXYZRGB& p, tf::StampedTransform t);
Point transformPoint(Point& p, tf::StampedTransform t);
bool continueLoop();
pcl::Normal findNormal(pcl::Normal n1, pcl::Normal n2);
void printQuaternion(Eigen::Quaterniond);


bool continueLoop(){
    char x;
    std::cout << "Continue? ";
    std::cin >> x;
    if (x=='n'){
        return false;
    }else{
        return true;
    }
}

void convertPoints(pcl::PointXYZRGB& p, Point& q){
    p.x = q.x;
    p.y = q.y;
    p.z = q.z;
}

void colorIt(pcl::PointXYZRGB& p, int color){
    uint8_t r=0,g=0,b=0;
    switch(color){
        case 0: r = 255; break;
        case 1: g = 255; break;
        case 2: b = 255; break;
        default : std::cerr<< "  What color?\n";
    }
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    p.rgb = *reinterpret_cast<float*>(&rgb);
}

tf::StampedTransform getTransform(std::string from_frame, std::string to_frame){
    //ROS_INFO("Finding transform from: %s to: %s",from_frame.c_str(), to_frame.c_str());
    tf::StampedTransform t;
    tf::TransformListener listener;
    try {
        listener.waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(from_frame, to_frame, ros::Time(0), t);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    return t;
}

pcl::PointXYZRGB transformPoint(pcl::PointXYZRGB& p, tf::StampedTransform t){
    tf::Vector3 p_temp(p.x, p.y, p.z);
    tf::Vector3 tp = t.inverse() * p_temp;
    pcl::PointXYZRGB q;
    q.x = tp[0];
    q.y = tp[1];
    q.z = tp[2];
    q.rgb = p.rgb;
    return q;
}

Point transformPoint(Point& p, tf::StampedTransform t){
    tf::Vector3 p_temp(p.x, p.y, p.z);
    tf::Vector3 tp = t.inverse() * p_temp;
    Point q;
    q.x = tp[0];
    q.y = tp[1];
    q.z = tp[2];
    return q;
}

void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t){
    tf::Vector3 origin = t.getOrigin();
    p.x = p.x - origin.x();
    p.y = p.y - origin.y();
    p.z = p.z - origin.z();
    //Translation done : Now rotation
}

void transformFromTo(Point& p, tf::StampedTransform t){
    tf::Vector3 origin = t.getOrigin();
    p.x = p.x - origin.x();
    p.y = p.y - origin.y();
    p.z = p.z - origin.z();
}

std::vector<int> findNearestPointIndices(pcl::PointXYZRGB& searchPoint, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZRGB>& kdtree, int K){
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        return pointIdxNKNSearch;
    }
    pointIdxNKNSearch.clear();
    return pointIdxNKNSearch;
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

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<cloud->width; ++i){
        pcl::PointXYZ new_point;
        new_point.x = cloud->points[i].x;
        new_point.y = cloud->points[i].y;
        new_point.z = cloud->points[i].z;
        new_cloud->points.push_back(new_point);
    }
    return new_cloud;
}

float frand(){
    return (float)rand()/(float)RAND_MAX;
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
#endif
