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
    void findFaceCenters(pcl::PointNormal &fc, pcl::Normal normal, bool direction);

    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ> cube_cloud;
    pcl::PointXYZ cube_center; // Cube center 
    pcl::Normal cube_axes[3]; // vectors along the sides. Any two would define the cube

    float scale;
    bool dense;

};
#endif
