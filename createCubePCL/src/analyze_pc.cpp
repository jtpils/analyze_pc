#include <createCubePCL/analyze_pc.h>

#define GT_CUBE "cube2"
#define QD_CUBE "cube1"
#define WORLD_FRAME "/world"
//#define CORRESPONDENCE_ONLY

void transformFromTo(geometry_msgs::Point& p, tf::StampedTransform t);
void transformFromTo(pcl::PointXYZRGB& p, tf::StampedTransform t);
tf::StampedTransform getTransform(std::string from_frame, std::string to_frame);

AnalyzePC::AnalyzePC():
gt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
qd_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    std::string gt_cloud_topic_name = "/"+(std::string)GT_CUBE+"/cloud";
    std::string qd_cloud_topic_name = "/"+(std::string)QD_CUBE+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &AnalyzePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &AnalyzePC::qdCloudCb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/cloud_vis_marker",1, this);
}

void AnalyzePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *gt_cloud);
}

void AnalyzePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *qd_cloud);
}

void AnalyzePC::visualizeError(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        return;
    }
    kdtree.setInputCloud(qd_cloud);
    visualization_msgs::Marker marker;
    marker.header.frame_id = WORLD_FRAME;
    marker.header.stamp = ros::Time();
    marker.ns = "cube_error";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.001;
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
    tf::StampedTransform tgq = getTransform(gt_cloud->header.frame_id, qd_cloud->header.frame_id);
    std_msgs::ColorRGBA c;

    if (gt_cloud->width == qd_cloud->width && gt_cloud->height == qd_cloud->height){
        for (size_t i=0; i<gt_cloud->width; ++i){
            geometry_msgs::Point p;
            p.x = gt_cloud->points[i].x;
            p.y = gt_cloud->points[i].y;
            p.z = gt_cloud->points[i].z;
            transformFromTo(p,tgw);
            marker.points.push_back(p);
            geometry_msgs::Point q;
            pcl::PointXYZRGB searchPoint = gt_cloud->points[i];
            transformFromTo(searchPoint, tgq);
            int K=1;
            std::vector<int> pointIdxNKNSearch(K);
            std::vector<float> pointNKNSquaredDistance(K);
            if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                q.x = qd_cloud->points[pointIdxNKNSearch[0]].x;
                q.y = qd_cloud->points[pointIdxNKNSearch[0]].y;
                q.z = qd_cloud->points[pointIdxNKNSearch[0]].z;
            }
            transformFromTo(q,tqw);
            // Not assuming exact correspondence
#ifdef CORRESPONDENCE_ONLY
            q.x = qd_cloud->points[i].x;
            q.y = qd_cloud->points[i].y;
            q.z = qd_cloud->points[i].z;
            transformFromTo(q,tqw);
#endif
            marker.points.push_back(q);
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
        c.r = 1.0*(error_data[i]-min_error)/(max_error-min_error);
        c.g = 0.0;
        c.b = 1.0*(max_error-error_data[i])/(max_error-min_error);
        c.a = 1.0;
        marker.colors.push_back(c);
        marker.colors.push_back(c);
        avg_error = (avg_error*i + error_data[i])/(i+1);
    }
    ROS_INFO("Average error is %f \n",avg_error);
    vis_pub.publish(marker);
}

void AnalyzePC::showKeyPoints(){
    pcl::HarrisKeypoint3D<PointT, PointT, NormalT> hkp;
    hkp.initCompute();
    pcl::PointCloud<pcl::PointXYZRGB> keypoints;
    hkp.setSearchSurface(gt_cloud);
    hkp.detectKeypoints(keypoints);
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
        loop_rate.sleep();
    }
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "analyze_pc");
    AnalyzePC test;
    test.spin();
    return 0;
}
