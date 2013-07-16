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
    PCLCube();
    void spin();
    void savetoFile();
    void savetoFile(std::string filename);
    void generatePoints();

  private:
    void generatePlanePoints(pcl::PointNormal, float);

    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ> cube_cloud;
    pcl::PointNormal cube_center; // Cube center and a vector along one of the faces
    float scale;
    bool dense;

};
#endif
