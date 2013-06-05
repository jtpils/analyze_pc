#include <createCubePCL/pcl_cube.h>
#include <visualization_msgs/Marker.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_keypoint3D.h>

class AnalyzePC {
  public:
    AnalyzePC();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher vis_pub;
    ros::Subscriber gt_cloud_sub;
    ros::Subscriber qd_cloud_sub;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr qd_cloud;
    std::vector<float> error_data;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

    void gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void visualizeError();
    void showKeyPoints();
};
