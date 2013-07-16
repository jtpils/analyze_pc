#ifndef COVERAGE_PC_H
#define COVERAGE_PC_H
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
/*
#include <visualization_msgs/Marker.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
*/
typedef pcl::PointXYZRGB Point;

enum correspondence {mBCL, mGCL, mQCL};

class CoveragePC {
  public:
    CoveragePC();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher coverage_cloud_pub;
    ros::Subscriber gt_cloud_sub;
    ros::Subscriber qd_cloud_sub;
    ros::ServiceServer set_parameters_server;

    pcl::PointCloud<Point>::Ptr gt_cloud;
    pcl::PointCloud<Point>::Ptr qd_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cov_cloud;
    std::vector<correspondence> cloud_corresp;

    void gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    bool setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void findCorrespondences();
    void estimateCoverage();
    float areaFunction(int, int f=0);

    double max_correspondence_distance;
    float cloud_fractions[3];
    int min_nn;
    double min_nn_factor;
    int test_number;
    bool data_generated;
};
#endif
