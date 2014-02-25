#ifndef PCL_CUBE_H
#define PCL_CUBE_H
/* File : pcl_cube.h
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube Pointcloud creation. PCLCube class declaration
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define DENSE_FACTOR 16
#define POINTS_PER_UNIT 32
#define NORMAL_SIGMA_FACTOR 2.0/POINTS_PER_UNIT/DENSE_FACTOR
#define INPLANE_SIGMA_FACTOR 2.0/POINTS_PER_UNIT/DENSE_FACTOR
#define CENTER_SIGMA_FACTOR 0.1
#define ORIENTATION_SIGMA_FACTOR 0.01

typedef boost::normal_distribution<double> NormalDist;
typedef boost::mt19937 RandomGen;
typedef boost::variate_generator<RandomGen& , NormalDist> GaussianGen;

class PCLCube{
  public:
    PCLCube(std::string name="cube"); // the constructor
    void spin(); // calls ros::spinOnce and hence the callbacks for subscriptions, if any
    void savetoFile(); //saves the cube point cloud to pcd file
    void savetoFile(std::string filename);
    void colorIt(uint8_t, uint8_t, uint8_t);
    void colorIt(uint32_t);
    void colorIt();
    void changeCenterTo(pcl::PointXYZ, bool world=false);
    void changeCenterBy(pcl::PointXYZ, bool world=false);
    void changeOrientationBy(Eigen::Quaterniond, bool world=false);
    void publishPointCloud(); // publishes the pcl point cloud after converting to sensor_msgs::PointCloud2
    void generatePoints(); //generates the points of the cube pointcloud
    void addNoise();
    void addNoise(GaussianGen&);
    void addNoiseToCenter();
    void addNoiseToCenter(GaussianGen&);
    void addNoiseToOrientation();
    void addNoiseToOrientation(GaussianGen&);
    void setOcclusion(float[]);
    tf::StampedTransform getFrameToWorldTransform();
    tf::StampedTransform getWorldToFrameTransform();

  public:
    void set_scale(double scale_) { scale = scale_; }
    void set_noise(bool noise_) {noise = noise_; }
    void set_center_noise(bool center_noise_) { center_noise = center_noise_; }
    void set_dense(bool dense_) { dense = dense_; }
    void set_dense_factor(int dense_factor_) { dense_factor = dense_factor_; }
    void set_points_per_unit(int points_per_unit_) { points_per_unit = points_per_unit_; }
    void set_normal_sigma_factor(double normal_sigma_factor_) { normal_sigma_factor = normal_sigma_factor_; }
    void set_inplane_sigma_factor(double inplane_sigma_factor_) { inplane_sigma_factor = inplane_sigma_factor_; }
    void set_center_sigma_factor(double center_sigma_factor_) { center_sigma_factor = center_sigma_factor_; }
    void set_orientation_sigma_factor(double orientation_sigma_factor_) { orientation_sigma_factor = orientation_sigma_factor_; }

    pcl::PointCloud<pcl::PointXYZRGB> get_cube_cloud() { return cube_cloud; }

  private:
    void generatePlanePoints(pcl::PointNormal, int); // given a face center, generate points in that plane
    pcl::PointNormal findFaceCenter(int index, bool direction); // finds the center of a face with normal in the direction of index'th normal
    double getGaussian(double, double);
    double getGaussian(double);
    void getTransform();
    bool regenerateCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::NodeHandle nh;
    //ros::AsyncSpinner _spinner; //TODO: ignore
    ros::Publisher point_cloud_pub; // ros publisher to  publish the cube cloud
    ros::ServiceServer regenerate_points_server;
    ros::ServiceServer get_parameters_server;
    tf::TransformListener listener;
    tf::StampedTransform world_to_frame_transform;
    pcl::PointCloud<pcl::PointXYZRGB> cube_cloud; // the cube cloud
    pcl::PointXYZ cube_center; // Cube center
    Eigen::Quaterniond cube_orientation;

    pcl::PointXYZ world_center;
    Eigen::Quaterniond world_orientation;

    pcl::Normal cube_axes[3]; // vectors along the sides. Any two would define the cube

    RandomGen rng;
    NormalDist gaussian_dist;
    GaussianGen *generator;

    double scale;
    bool dense;
    bool noise;
    bool center_noise;
    uint32_t rgb;
    std::string cube_name;
    int dense_factor;
    int points_per_unit;
    double normal_sigma_factor;
    double inplane_sigma_factor;
    double center_sigma_factor;
    double orientation_sigma_factor;
    float occluded_fraction[6];

};
#endif
