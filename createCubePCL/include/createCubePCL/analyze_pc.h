#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ Point;
class AnalyzePC {
  public:
    AnalyzePC();
    void spin();

  private:
    ros::NodeHandle nh;
    ros::Publisher vis_pub;
    ros::Publisher kpg_pub;
    ros::Publisher kpq_pub;
    ros::Publisher registered_cloud_pub;
    ros::Publisher registered_kp_pub;
    ros::Subscriber gt_cloud_sub;
    ros::Subscriber qd_cloud_sub;
    ros::ServiceServer set_parameters_server;
    pcl::PointCloud<Point>::Ptr gt_cloud;
    pcl::PointCloud<Point>::Ptr qd_cloud;
    pcl::PointCloud<Point>::Ptr transformed_qd_cloud;
    std::vector<float> error_data;

    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_gt;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_qd;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_gt;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_qd;
    pcl::visualization::PCLHistogramVisualizer hist;
    pcl::SampleConsensusInitialAlignment<Point, Point, pcl::FPFHSignature33> sac_ia;
    Eigen::Matrix4f transformation_q_g;
    float min_fitness_score;

    void gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    void qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input);
    bool setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void visualizeError();
    void showKeyPoints(bool cache=false);
    void estimateFPFHFeatures(bool cache=false);
    pcl::PointCloud<Point>::Ptr getCloud(std::string);
    void applySACIA();

    double harris_radius;
    double normal_estimation_radius;
    double fpfh_estimation_radius;
    double min_sample_distance;
    double max_correspondence_distance;
    int nr_iterations;

    bool feature_added_gt;
    bool feature_added_qd;
};
