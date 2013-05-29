#ifndef PCL_CUBE_H
#define PCL_CUBE_H
/* File : pcl_cube.h
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube Pointcloud creation. PCLCube class declaration
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define DENSE_FACTOR 16
#define POINTS_PER_UNIT 32
#define NORMAL_SIGMA_FACTOR 2.0/POINTS_PER_UNIT/DENSE_FACTOR
#define INPLANE_SIGMA_FACTOR 2.0/POINTS_PER_UNIT/DENSE_FACTOR
#define CENTER_SIGMA_FACTOR 1/10

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
    void changeCenterTo(pcl::PointXYZ);
    void publishPointCloud(); // publishes the pcl point cloud after converting to sensor_msgs::PointCloud2
    void generatePoints(); //generates the points of the cube pointcloud
    void addNoise();
    void addNoise(GaussianGen&);
    void addNoiseToCenter();
    void addNoiseToCenter(GaussianGen&);

  private:
    void generatePlanePoints(pcl::PointNormal, int); // given a face center, generate points in that plane
    pcl::PointNormal findFaceCenter(int index, bool direction); // finds the center of a face with normal in the direction of index'th normal
    double getGaussian(double, double);
    double getGaussian(double);

    ros::NodeHandle nh;
    //ros::AsyncSpinner _spinner; //TODO: ignore
    ros::Publisher point_cloud_pub; // ros publisher to  publish the cube cloud
    pcl::PointCloud<pcl::PointXYZRGB> cube_cloud; // the cube cloud
    pcl::PointXYZ cube_center; // Cube center 
    pcl::Normal cube_axes[3]; // vectors along the sides. Any two would define the cube

    RandomGen rng;
    NormalDist gaussian_dist;
    GaussianGen *generator;

    float scale;
    bool dense;
    bool noise;
    std::string cube_name;

};
#endif
