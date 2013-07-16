#include <createCubePCL/analyze_pc.h>
#include "dm_colors.hpp"

#define WORLD_FRAME "/world"
//#define CORRESPONDENCE_ONLY
//#define ERROR_LINES_DISPLAY
//#define VIEW_FPFH_HISTOGRAMS
#define SAVE_FPFH_HISTOGRAMS

void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t);
void transformFromTo(Point& p, tf::StampedTransform t);
tf::StampedTransform getTransform(std::string from_frame, std::string to_frame);
std::vector<int> findNearestPointIndices(Point searchPoint, pcl::PointCloud<Point>::Ptr cloud, int K=1);
pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

std::string gt_name;
std::string qd_name;

AnalyzePC::AnalyzePC():
gt_cloud(new pcl::PointCloud<Point>),
qd_cloud(new pcl::PointCloud<Point>),
keypoints_gt(new pcl::PointCloud<pcl::PointXYZI>),
keypoints_qd(new pcl::PointCloud<pcl::PointXYZI>)
{
    std::string gt_cloud_topic_name = "/"+gt_name+"/cloud";
    std::string qd_cloud_topic_name = "/"+qd_name+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &AnalyzePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &AnalyzePC::qdCloudCb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/cloud_vis_marker",1, this);
    kpg_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+gt_name+"/kp_cloud",1);
    kpq_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+qd_name+"/kp_cloud",1);
    set_parameters_server = nh.advertiseService("/analyze_pc/set_parameters",
            &AnalyzePC::setParamCb, this);

    harris_radius = 0.01;
    nh.setParam("/analyze_pc/harris_radius", harris_radius);
    normal_estimation_radius = 0.01;
    nh.setParam("/analyze_pc/normal_estimation_radius", normal_estimation_radius);
    fpfh_estimation_radius = 0.03;
    nh.setParam("/analyze_pc/fpfh_estimation_radius", fpfh_estimation_radius);

    feature_added_gt = false;
    feature_added_qd = false;
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
            Point searchPoint = qd_cloud->points[i];
            transformFromTo(searchPoint, tqg);
            // Not assuming exact correspondence
            int point_index = findNearestPointIndices(searchPoint, qd_cloud, 1)[0];
            q.x = gt_cloud->points[point_index].x;
            q.y = gt_cloud->points[point_index].y;
            q.z = gt_cloud->points[point_index].z;
            transformFromTo(q,tgw);
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
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp;
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    sensor_msgs::PointCloud2 kp_pc;
    hkp.setRadius(harris_radius);
    hkp.setSearchMethod(tree);

    hkp.setInputCloud(gt_cloud);
    hkp.compute(*keypoints_gt);
    keypoints_gt->header.frame_id=gt_cloud->header.frame_id;
    pcl::toROSMsg(*keypoints_gt, kp_pc);
    kpg_pub.publish(kp_pc);
    pcl::io::savePCDFileASCII ("kp_gt.pcd", *keypoints_gt);
    ROS_INFO("Found GT_CLOUD keypoints :%d", keypoints_gt->points.size());

    hkp.setInputCloud(qd_cloud);
    hkp.compute(*keypoints_qd);
    keypoints_qd->header.frame_id=qd_cloud->header.frame_id;
    pcl::toROSMsg(*keypoints_qd, kp_pc);
    kpq_pub.publish(kp_pc);
    pcl::io::savePCDFileASCII ("kp_qd.pcd", *keypoints_qd);
    ROS_INFO("Found QD_CLOUD keypoints :%d", keypoints_qd->points.size());
}

void AnalyzePC::estimateFPFHFeatures(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Estimating FPFH Features");
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    pcl::FPFHEstimationOMP<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimation;

    normal_estimation.setInputCloud(gt_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_estimation_radius);
    normal_estimation.compute(*normals);
    fpfh.setSearchSurface(gt_cloud);
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(toPointXYZ(keypoints_gt));
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_estimation_radius);
    fpfh.compute(fpfhs_gt);

    ROS_INFO("Found GT_CLOUD feature histogram :%d",fpfhs_gt.size());
#ifdef SAVE_FPFH_HISTOGRAMS
    pcl::io::savePCDFileASCII ("fpfhs_gt.pcd", fpfhs_gt);
#endif

#ifdef VIEW_FPFH_HISTOGRAMS
    if (!feature_added_gt){
        hist.addFeatureHistogram (fpfhs_gt, 33 , "fpfh_dist_gt", 640, 200);
        feature_added_gt= true;
    }else{
        hist.updateFeatureHistogram (fpfhs_gt, 33 , "fpfh_dist_gt");
    }
#endif

    normal_estimation.setInputCloud(qd_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_estimation_radius);
    normal_estimation.compute(*normals);
    fpfh.setSearchSurface(qd_cloud);
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(toPointXYZ(keypoints_qd));
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_estimation_radius);
    fpfh.compute(fpfhs_qd);

    ROS_INFO("Found QD_CLOUD feature histogram :%d", fpfhs_qd.points.size());
#ifdef SAVE_FPFH_HISTOGRAMS
    pcl::io::savePCDFileASCII ("fpfhs_qd.pcd", fpfhs_qd);
#endif

#ifdef VIEW_FPFH_HISTOGRAMS
    if (!feature_added_qd){
        hist.addFeatureHistogram (fpfhs_qd, 33 , "fpfh_dist_qd", 640, 200);
        feature_added_qd = true;
    }else{
        hist.updateFeatureHistogram (fpfhs_qd, 33 , "fpfh_dist_qd");
    }
#endif
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

void transformFromTo(Point& p, tf::StampedTransform t){
    tf::Vector3 origin = t.getOrigin();
    p.x = p.x - origin.x();
    p.y = p.y - origin.y();
    p.z = p.z - origin.z();
}

std::vector<int> findNearestPointIndices(Point searchPoint, pcl::PointCloud<Point>::Ptr cloud,
        int K){
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        return pointIdxNKNSearch;
    }
    pointIdxNKNSearch.clear();
    return pointIdxNKNSearch;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<cloud->width; ++i){
        pcl::PointXYZ new_point;
        new_point.x = cloud->points[i].x;
        new_point.y = cloud->points[i].y;
        new_point.z = cloud->points[i].z;
        new_cloud->points.push_back(new_point);
    }
    return new_cloud;
}

void AnalyzePC::spin(){
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        //visualizeError();
        showKeyPoints();
        estimateFPFHFeatures();
#ifdef VIEW_FPFH_HISTOGRAMS
        hist.spinOnce(10);
#endif
        ROS_INFO(" ");
        loop_rate.sleep();
    }
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "analyze_pc");
    gt_name = "cube2";
    qd_name = "cube1";
    if (argc == 3){
        gt_name = argv[1];
        qd_name = argv[2];
    }
    AnalyzePC test;
    test.spin();
    return 0;
}
