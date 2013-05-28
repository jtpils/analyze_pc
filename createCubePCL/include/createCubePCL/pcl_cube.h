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

#define DENSE_FACTOR 16
#define POINTS_PER_UNIT 128

class PCLCube{
  public:
    PCLCube(std::string name="cube"); // the constructor
    void spin(); // calls ros::spinOnce and hence the callbacks for subscriptions, if any
    void savetoFile(); //saves the cube point cloud to pcd file
    void savetoFile(std::string filename);
    void colorIt(uint8_t, uint8_t, uint8_t);
    void publishPointCloud(); // publishes the pcl point cloud after converting to sensor_msgs::PointCloud2

  private:
    void generatePlanePoints(pcl::PointNormal, int); // given a face center, generate points in that plane
    pcl::PointNormal findFaceCenter(int index, bool direction); // finds the center of a face with normal in the direction of index'th normal

    ros::NodeHandle nh;
    //ros::AsyncSpinner _spinner; //TODO: ignore
    ros::Publisher point_cloud_pub; // ros publisher to  publish the cube cloud
    pcl::PointCloud<pcl::PointXYZRGB> cube_cloud; // the cube cloud
    pcl::PointXYZ cube_center; // Cube center 
    pcl::Normal cube_axes[3]; // vectors along the sides. Any two would define the cube
    void generatePoints(); //generates the points of the cube pointcloud

    float scale;
    bool dense;
    std::string cube_name;

};
#endif
