
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include "analyze_pc/pcl_cube.h"
#include "dm_colors.hpp"

//void eval_density() {
  //float cloud_fractions[3];

  //pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_cov;
  //kdtree_cov.setInputCloud(cov_cloud);

  //std::vector<int> k_indices;
  //std::vector<float> k_sqr_distances;
  //pcl::PointXYZRGB focusPoint;

  //int no_of_points;
  //int corr_np[3]={0,0,0};
  //for (int i=0; i<3; ++i){
    //cloud_fractions[i] = 0;
  //}
  //float max_correspondence_distance = 1.2;

  //float sum_area[3] = {0,0,0};
  //for (size_t i=0; i<cov_cloud->size(); ++i){
    //focusPoint = cov_cloud->points[i];
    //no_of_points = kdtree_cov.radiusSearch(focusPoint, max_correspondence_distance, k_indices, k_sqr_distances);
    //if (no_of_points>0){
      //cloud_fractions[(int)cloud_corresp[i]] += areaFunction(no_of_points);
      //for (int j=0; j<3; ++j){
        //sum_area[j]+=areaFunction(no_of_points, j);
      //}
    //}
    //corr_np[(int)cloud_corresp[i]]++;
  //}
//}

template<class T>
float float_to_pcl_rgb(T r, T g, T b) {
  uint32_t ri = static_cast<uint32_t>(r*255);
  uint32_t gi = static_cast<uint32_t>(g*255);
  uint32_t bi = static_cast<uint32_t>(b*255);
  uint32_t rgb = (ri<<16)|(gi<<8)|bi;
  return (*reinterpret_cast<float*>(&rgb));
}

void eval_density(pcl::PointCloud<pcl::PointXYZRGB>& pc) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcptr(new pcl::PointCloud<pcl::PointXYZRGB>(pc));
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_cov;
  kdtree_cov.setInputCloud(pcptr);

  float minval = 1e6;
  float maxval = -1;
  std::vector<int> k_indices;
  std::vector<float> k_sqr_distances;
  //float max_correspondence_distance = 1.2;
  float max_correspondence_distance = 0.1;
  for (size_t i=0; i<pcptr->size(); ++i){
    //if (i % 10 != 0) { continue; }
    if (i % 100 == 0) {
      std::cerr << "pt " << i << "/" << pcptr->size() << std::endl;
    }
    const pcl::PointXYZRGB& focusPoint(pcptr->points[i]);

    int no_of_points = kdtree_cov.radiusSearch(focusPoint, max_correspondence_distance, k_indices, k_sqr_distances);
    k_indices.clear();
    k_sqr_distances.clear();

    if (i % 100 == 0) {
      std::cerr << no_of_points << std::endl;
    }

    if (no_of_points>0){
      float inv_density = 1./no_of_points;
      float rgb[3];
      minval = std::min(inv_density, minval);
      maxval = std::max(inv_density, maxval);
      //std::cerr << "minval = " << minval << std::endl;
      //std::cerr << "maxval = " << maxval << std::endl;
      //dm::colormap<float>(inv_density, 8e-5, 0.002, dm::jet_colormap, rgb);
      dm::colormap<float>(inv_density, 0.0003, 0.0099, dm::jet_colormap, rgb);
      //focusPoint.rgb = float_to_pcl_rgb(rgb[0], rgb[1], rgb[2]);
      //std::cerr << "inv_density = " << inv_density << std::endl;
      //std::cerr << "rgb = " << ", " << rgb[0] << ", " << rgb[1] << ", " << rgb[2] << std::endl;
      pc.points[i].rgb = float_to_pcl_rgb(rgb[0], rgb[1], rgb[2]);
    }

  }

}

void make_cube(pcl::PointCloud<pcl::PointXYZRGB>& pc) {

  PCLCube cube1;
  cube1.set_dense(true);
  cube1.set_dense_factor(8);
  cube1.set_points_per_unit(32);
  cube1.generatePoints();
  cube1.savetoFile("cube1.pcd");
  pcl::PointCloud<pcl::PointXYZRGB> pc1 = cube1.get_cube_cloud();

  PCLCube cube2;
  cube2.set_dense(true);
  cube2.set_dense_factor(2); cube2.set_points_per_unit(32);
  cube2.generatePoints();
  cube2.savetoFile("cube2.pcd");
  pcl::PointCloud<pcl::PointXYZRGB> pc2 = cube2.get_cube_cloud();

  //pcl::PointCloud<pcl::PointXYZRGB> pc;
  for (size_t i=0; i < pc1.size(); ++i) {
    if (pc1[i].y < 0) { pc.push_back(pc1[i]); }
  }
  for (size_t i = 0; i < pc2.size(); ++i) {
    if (pc1[i].y > 0) { pc.push_back(pc2[i]); }
  }

  pcl::io::savePCDFile("halfhalf.pcd", pc);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pcd_to_pointcloud2");

  pcl::PointCloud<pcl::PointXYZRGB> pc;
  make_cube(pc);

  eval_density(pc);

  pcl::io::savePCDFile("halfhalf_inv_dens.pcd", pc);
  return 0;
}
