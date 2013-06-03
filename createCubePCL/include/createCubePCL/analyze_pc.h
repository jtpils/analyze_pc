#include <createCubePCL/pcl_cube.h>
#include <visualization_msgs/Marker.h>
#include <pcl/kdtree/kdtree_flann.h>

class AnalyzePC {
  public:
    AnalyzePC();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher vis_pub;
    ros::Subscriber gt_cloud_sub;
    ros::Subscriber qd_cloud_sub;
    pcl::PointCloud<pcl::PointXYZRGB> gt_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> qd_cloud;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    void gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void visualizeError();
};
