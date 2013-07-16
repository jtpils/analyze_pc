#include <createCubePCL/analyze_pc.h>
#include "dm_colors.hpp"

#define GT_CLOUD "cube2"
#define QD_CLOUD "cube1"
#define WORLD_FRAME "/world"
//#define CORRESPONDENCE_ONLY
//#define ERROR_LINES_DISPLAY

void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t);
void transformFromTo(pcl::PointXYZRGB& p, tf::StampedTransform t);
tf::StampedTransform getTransform(std::string from_frame, std::string to_frame);

AnalyzePC::AnalyzePC():
gt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
qd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    std::string gt_cloud_topic_name = "/"+(std::string)GT_CLOUD+"/cloud";
    std::string qd_cloud_topic_name = "/"+(std::string)QD_CLOUD+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &AnalyzePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &AnalyzePC::qdCloudCb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/cloud_vis_marker",1, this);
    kpg_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+(std::string)GT_CLOUD+"/kp_cloud",1);
    kpq_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+(std::string)QD_CLOUD+"/kp_cloud",1);
    set_parameters_server = nh.advertiseService("/analyze_pc/set_parameters",
            &AnalyzePC::setParamCb, this);

    harris_radius = 0.3;
    nh.setParam("/analyze_pc/harris_radius", harris_radius);
    normal_estimation_radius = 0.2;
    nh.setParam("/analyze_pc/normal_estimation_radius", normal_estimation_radius);
    fpfh_estimation_radius = 0.4;
    nh.setParam("/analyze_pc/fpfh_estimation_radius", fpfh_estimation_radius);

    feature_added = false;
}

void AnalyzePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *gt_cloud);
}

void AnalyzePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *qd_cloud);
}

void AnalyzePC::visualizeError(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Visualizing the error");
    kdtree.setInputCloud(gt_cloud);
    visualization_msgs::Marker marker;
    marker.header.frame_id = WORLD_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "cube_error";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
#ifdef ERROR_LINES_DISPLAY
    marker.type = visualization_msgs::Marker::LINE_LIST;
#endif
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
#ifdef ERROR_LINES_DISPLAY
    marker.scale.x = 0.001;
#endif
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.points.clear();
    marker.colors.clear();
    error_data.clear();
    float min_error = FLT_MAX;
    float max_error = 0.0;

    tf::StampedTransform tgw = getTransform(gt_cloud->header.frame_id, WORLD_FRAME);
    tf::StampedTransform tqw = getTransform(qd_cloud->header.frame_id, WORLD_FRAME);
    tf::StampedTransform tqg = getTransform(qd_cloud->header.frame_id, gt_cloud->header.frame_id);
    std_msgs::ColorRGBA c;

    if (gt_cloud->width == qd_cloud->width && gt_cloud->height == qd_cloud->height){
        for (size_t i=0; i<qd_cloud->width; ++i){
            geometry_msgs::Point p;
            p.x = qd_cloud->points[i].x;
            p.y = qd_cloud->points[i].y;
            p.z = qd_cloud->points[i].z;
            transformFromTo(p,tqw);
            marker.points.push_back(p);
            geometry_msgs::Point q;
            pcl::PointXYZRGB searchPoint = qd_cloud->points[i];
            transformFromTo(searchPoint, tqg);
            int K=1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                q.x = gt_cloud->points[pointIdxNKNSearch[0]].x;
                q.y = gt_cloud->points[pointIdxNKNSearch[0]].y;
                q.z = gt_cloud->points[pointIdxNKNSearch[0]].z;
            }
            transformFromTo(q,tgw);
            // Not assuming exact correspondence
#ifdef CORRESPONDENCE_ONLY
            q.x = gt_cloud->points[i].x;
            q.y = gt_cloud->points[i].y;
            q.z = gt_cloud->points[i].z;
            transformFromTo(q,tgw);
#endif
#ifdef ERROR_LINES_DISPLAY
            marker.points.push_back(q);
#endif
            float error = (p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y)+(p.z-q.z)*(p.z-q.z);
            if (error > max_error){
                max_error = error;
            }
            if (error < min_error){
                min_error = error;
            }
            error_data.push_back(error);
        }
    }
    else{
        ROS_ERROR("PointClouds are not registered");
    }
    float avg_error = 0.0;
    for (size_t i=0; i<error_data.size(); ++i){
        float rgb[3];
        dm::colormap(error_data[i], min_error, max_error, dm::line_colormap, rgb);
        c.r = rgb[0];
        c.g = rgb[1];
        c.b = rgb[2];
        c.a = 1.0;
        marker.colors.push_back(c);
#ifdef ERROR_LINES_DISPLAY
        marker.colors.push_back(c);
#endif
        avg_error = (avg_error*i + error_data[i])/(i+1);
    }
    ROS_INFO("Average error is %f",avg_error);
    vis_pub.publish(marker);
}

void AnalyzePC::showKeyPoints(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Finding the keypoints");
    pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::PointNormal> hkp;
    sensor_msgs::PointCloud2 kp_pc;
    hkp.setRadius(harris_radius);

    hkp.setInputCloud(gt_cloud);
    hkp.compute(keypoints_gt);
    keypoints_gt.header.frame_id=gt_cloud->header.frame_id;
    pcl::toROSMsg(keypoints_gt, kp_pc);
    kpg_pub.publish(kp_pc);

    hkp.setInputCloud(qd_cloud);
    hkp.compute(keypoints_qd);
    keypoints_qd.header.frame_id=qd_cloud->header.frame_id;
    pcl::toROSMsg(keypoints_qd, kp_pc);
    kpq_pub.publish(kp_pc);
}

void AnalyzePC::estimateFPFHFeatures(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Estimating FPFH Features");
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    normal_estimation.setInputCloud(gt_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_estimation_radius);
    normal_estimation.compute(*normals);
    fpfh.setInputCloud(gt_cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_estimation_radius);
    fpfh.compute(fpfhs_gt);

    normal_estimation.setInputCloud(qd_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_estimation_radius);
    normal_estimation.compute(*normals);
    fpfh.setInputCloud(qd_cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_estimation_radius);
    fpfh.compute(fpfhs_qd);

    if (!feature_added){
        hist.addFeatureHistogram (fpfhs_gt, 33 , "fpfh_dist_gt", 640, 200);
        hist.addFeatureHistogram (fpfhs_qd, 33 , "fpfh_dist_qd", 640, 200);
        feature_added = true;
    }else{
        hist.updateFeatureHistogram (fpfhs_gt, 33 , "fpfh_dist_gt");
        hist.updateFeatureHistogram (fpfhs_qd, 33 , "fpfh_dist_qd");
    }
}

bool AnalyzePC::setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    nh.getParam("/analyze_pc/harris_radius", harris_radius);
    nh.getParam("/analyze_pc/normal_estimation_radius", normal_estimation_radius);
    nh.getParam("/analyze_pc/fpfh_estimation_radius", fpfh_estimation_radius);
    return true;
}

tf::StampedTransform getTransform(std::string from_frame, std::string to_frame){
    tf::StampedTransform t;
    tf::TransformListener listener;
    try {
        listener.waitForTransform(from_frame, to_frame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform(from_frame, to_frame, ros::Time(0), t);
    }catch(tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    return t;
}

void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t){
    tf::Vector3 origin = t.getOrigin();
    p.x = p.x - origin.x();
    p.y = p.y - origin.y();
    p.z = p.z - origin.z();
    //Translation done : Now rotation
}

void transformFromTo(pcl::PointXYZRGB& p, tf::StampedTransform t){
    tf::Vector3 origin = t.getOrigin();
    p.x = p.x - origin.x();
    p.y = p.y - origin.y();
    p.z = p.z - origin.z();
}

void AnalyzePC::spin(){
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        visualizeError();
        showKeyPoints();
        estimateFPFHFeatures();
        hist.spinOnce(10);
        ROS_INFO(" ");
        loop_rate.sleep();
    }
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "analyze_pc");
    AnalyzePC test;
    test.spin();
    return 0;
}
