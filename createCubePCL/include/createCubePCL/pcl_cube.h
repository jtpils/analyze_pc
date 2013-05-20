/* File : create_cube.h
 * Author : Savant Krishna <savant.2020@gmail.com>
 * Description : Cube Pointcloud creation. PCLCube class declaration
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class PCLCube{
  public:
    PCLCube();
    void savetoFile();
    void savetoFile(std::string filename);

  private:
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ> cube_cloud;
    pcl::PointXYZ cube_center;

    void generatePoints();

    bool dense;

};
