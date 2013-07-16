#include <createCubePCL/pcl_cube.h>

class AnalyzePC {
  public:
    AnalyzePC();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Subscriber gt_cloud_sub;
    ros::Subscriber er_cloud_sub;
    pcl::PointCloud<pcl::PointXYZRGB> gt_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> er_cloud;

    void gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void erCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
};
