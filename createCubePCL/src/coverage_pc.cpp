#include <createCubePCL/coverage_pc.h>
#include <boost/filesystem.hpp>
#include "dm_colors.hpp"
#include <fstream>

#define WORLD_FRAME "/world"
#define PRINT_NN_DATA
//#define SAVE_COV_DATA

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud, pcl::KdTreeFLANN<Point>& kdtree, int K);
void convertPoints(pcl::PointXYZRGB& p, Point& q);
void colorIt(pcl::PointXYZRGB& p, int color);
Point transformPoint(Point& p, tf::StampedTransform t);
tf::StampedTransform getTransform(std::string from_frame, std::string to_frame);
bool continueLoop();

std::string qd_name;
std::string gt_name;
float md,vd;
int cutoff_nn;
double avg_occ;

CoveragePC::CoveragePC():
gt_cloud(new pcl::PointCloud<Point>),
qd_cloud(new pcl::PointCloud<Point>),
cov_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    std::string gt_cloud_topic_name = "/"+gt_name+"/cloud";
    std::string qd_cloud_topic_name = "/"+qd_name+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &CoveragePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &CoveragePC::qdCloudCb, this);
    coverage_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/coverage_pc/"+gt_name+"_"+qd_name+"_coverage",1);
    set_parameters_server = nh.advertiseService("/coverage_pc/set_parameters", &CoveragePC::setParamCb, this);
    max_correspondence_distance = 0.05;
    nh.setParam("/coverage_pc/max_correspondence_distance", max_correspondence_distance);
    min_nn = 10;
    nh.setParam("/coverage_pc/min_nn", min_nn);
    min_nn_factor = 0.8;
    nh.setParam("/coverage_pc/min_nn_factor", min_nn_factor);
    test_number = 0;
    nh.setParam("/coverage_pc/test_number", test_number);
    cutoff_nn = 5;
    nh.setParam("/coverage_pc/cutoff_nn", cutoff_nn);
    data_generated = false;
#ifdef SAVE_COV_DATA
    data_generated = true;
#endif
#ifdef PRINT_NN_DATA
    data_generated = true;
#endif
}

void CoveragePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *gt_cloud);
}

void CoveragePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *qd_cloud);
}

bool CoveragePC::setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    nh.getParam("/coverage_pc/max_correspondence_distance", max_correspondence_distance);
    nh.getParam("/coverage_pc/min_nn", min_nn);
    nh.getParam("/coverage_pc/min_nn_factor", min_nn_factor);
    nh.getParam("/coverage_pc/test_number", test_number);
    nh.getParam("/coverage_pc/cutoff_nn", cutoff_nn);
#ifdef PRINT_NN_DATA
    data_generated = false;
#endif
#ifdef SAVE_COV_DATA
    data_generated = false;
#endif
    return true;
}

void CoveragePC::findCorrespondences(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_WARN("Not yet received point clouds");
        return;
    }
    //ROS_INFO("Find correspondences between pointclouds");
    pcl::KdTreeFLANN<Point> kdtree_gt;
    pcl::KdTreeFLANN<Point> kdtree_qd;
    kdtree_gt.setInputCloud(gt_cloud);
    kdtree_qd.setInputCloud(qd_cloud);
    std::vector<bool> gt_covered;
    std::vector<bool> qd_covered;
    gt_covered.clear();
    qd_covered.clear();
    cloud_corresp.clear();
    cov_cloud->points.clear();
    std::string gt_frame = gt_cloud->header.frame_id;
    std::string qd_frame = qd_cloud->header.frame_id;
    float distance;
    tf::StampedTransform tgw = getTransform(gt_frame, WORLD_FRAME);
    tf::StampedTransform tqg = getTransform(qd_frame, gt_frame);
    tf::StampedTransform tgq = getTransform(gt_frame, qd_frame);
    //tf::StampedTransform tgq = tqg.inverse();
    tf::StampedTransform tqw = getTransform(qd_frame, WORLD_FRAME);
    for (size_t i=0; i<gt_cloud->size(); ++i){
        gt_covered.push_back(false);
    }
    int counter=0;
    float sum_distance = 0;
    std::vector<float> distance_array;
    for (size_t i=0; i<qd_cloud->size(); ++i){
        pcl::PointXYZRGB point;
        Point searchPoint = transformPoint(qd_cloud->points[i], tqg);
        int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree_gt, 1)[0];
        Point resultPoint = gt_cloud->points[point_index];
        Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
        Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
        distance = (p1-p2).squaredNorm();
        if (distance < max_correspondence_distance*max_correspondence_distance){
            if (!gt_covered[point_index]){
                gt_covered[point_index] = true;
                convertPoints(point, resultPoint);
                colorIt(point, 0);
                cov_cloud->points.push_back(transformPoint(point, tgw));
                counter++;
                cloud_corresp.push_back(mBCL);
        sum_distance += sqrt(distance);
        distance_array.push_back(sqrt(distance));
            }
            qd_covered.push_back(true);
            convertPoints(point, searchPoint);
            colorIt(point, 0);
            counter++;
            //cov_cloud->points.push_back(transformPoint(point, tgw));
            cloud_corresp.push_back(mBCL);
        }else{
            qd_covered.push_back(false);
            convertPoints(point, searchPoint);
            colorIt(point, 2);
            counter++;
            //cov_cloud->points.push_back(transformPoint(point, tgw));
            cloud_corresp.push_back(mQCL);
        }
    }
    for (size_t i=0; i<gt_cloud->size(); ++i){
        pcl::PointXYZRGB point;
        Point searchPoint = transformPoint(gt_cloud->points[i], tgq);
        int rev_index = findNearestPointIndices(searchPoint, qd_cloud, kdtree_qd, 1)[0];
        Point resultPoint = qd_cloud->points[rev_index];
        Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
        Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
        distance = (p1-p2).squaredNorm();
        if (!gt_covered[i]){
            convertPoints(point, searchPoint);
            if (distance < max_correspondence_distance*max_correspondence_distance){
                gt_covered[i] = true;
                colorIt(point, 0);
                cloud_corresp.push_back(mBCL);
                sum_distance += sqrt(distance);
                distance_array.push_back(sqrt(distance));
            }else{
                colorIt(point, 1);
                cloud_corresp.push_back(mGCL);
            }
            cov_cloud->points.push_back(transformPoint(point, tqw));
            counter++;
        }
    }

    cov_cloud->header.frame_id = WORLD_FRAME;
    cov_cloud->height = 1;
    cov_cloud->width = cov_cloud->size();
    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg(*cov_cloud, pc);
    //std::cerr << "coverage cloud :" << cov_cloud->size() << "\n";
    coverage_cloud_pub.publish(pc);

    md = sum_distance/cov_cloud->size();
    ROS_INFO("Mean distance from nearest neighbor is : %f", md);
    float sum_sq_diff=0;
    for (size_t i=0; i<distance_array.size(); ++i){
        sum_sq_diff += (distance_array[i]-md)*(distance_array[i]-md);
    }
    vd = sum_sq_diff/cov_cloud->size();
    ROS_INFO("Variance of distance from nearest neighbor is : %f", vd);
}

float CoveragePC::areaFunction(int n, int function){
    float K = min_nn_factor*min_nn_factor*min_nn*min_nn*min_nn*(1-min_nn_factor);
    switch (function){
        case 0: return 1.0/(float)n; break;
        case 1:
            if(n<cutoff_nn){
                return 0;
            }else{
                return 1.0/(float)n;
            }
            break;
        case 2:
            if (n < min_nn_factor*min_nn){
                return n*(min_nn-n)/K;
            }else{
                return 1.0/(float)n;
            }
            break;
    }
}

// Estimating coverage after establishing which part each point belongs to
// For each point, find the local density and give area corresponding to it
// to it
void CoveragePC::estimateCoverage(){
#ifdef SAVE_COV_DATA
    std::ofstream resout;
    boost::filesystem::create_directories("./res/");
    resout.open("res/res_data.txt", std::ofstream::out | std::ofstream::app);
    resout << min_nn_factor << " " << min_nn << "\n";
#endif
#ifdef PRINT_NN_DATA
    std::ofstream fout[3];
    boost::filesystem::create_directories("./dist/"+boost::to_string(test_number)+"/");
    for (int i=0; i<3; ++i){
        fout[i].open(("dist/"+boost::to_string(test_number)+"/"+boost::to_string(i)+"_data.txt").c_str(), std::ofstream::out);
    }
#endif
    int count =0;
    for (size_t i=0; i<cov_cloud->size(); ++i){
        if (cloud_corresp[i] == mBCL){
            count++;
        }
    }
#ifdef PRINT_NN_DATA
    std::ofstream fro;
    fro.open("dist/fraction_data.txt", std::ofstream::out | std::ofstream::app);
    fro << max_correspondence_distance << "\n";
    //fro << md << " " << vd << " " << md/count << " ";
    nh.getParam("/compare_two_cubes/avg_occ", avg_occ);
    fro << avg_occ << " ";
#endif
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_WARN("Not yet received point clouds");
        return;
    }
    ROS_INFO("Estimating Coverage of pointclouds");
    ROS_INFO("No. of points are : %d %d %d", gt_cloud->size(), qd_cloud->size(), cov_cloud->size());
    pcl::KdTreeFLANN<Point> kdtree_cov;
    kdtree_cov.setInputCloud(cov_cloud);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    Point focusPoint;
    int no_of_points;
    int corr_np[3]={0,0,0};
    for (int i=0; i<3; ++i){
        cloud_fractions[i] = 0;
    }
    float sum_area[3] = {0,0,0};
    for (size_t i=0; i<cov_cloud->size(); ++i){
        focusPoint = cov_cloud->points[i];
        no_of_points = kdtree_cov.radiusSearch(focusPoint, max_correspondence_distance, k_indices, k_sqr_distances);
        if (no_of_points>0){
            cloud_fractions[(int)cloud_corresp[i]] += areaFunction(no_of_points);
            for (int j=0; j<3; ++j){
                sum_area[j]+=areaFunction(no_of_points, j);
            }
        }
        corr_np[(int)cloud_corresp[i]]++;
#ifdef PRINT_NN_DATA
        fout[cloud_corresp[i]] << no_of_points << "\n";
#endif
    }
    std::cerr << "Area estimates : ";
    for (int i=0; i<3; ++i){
        std::cerr << sum_area[i] << " ";
    }
    std::cerr << "\n";
#ifdef SAVE_COV_DATA
    for (int i=0; i<3; ++i){
        resout << corr_np[i] << " ";
    }
    resout << "\n";
#endif
    float sum_fractions=0.0;
    std::cerr << "fractions : ";
    for (int i=0; i<3; ++i){
        sum_fractions+=cloud_fractions[i];
        std::cerr << cloud_fractions[i] << " ";
#ifdef SAVE_COV_DATA
        resout << cloud_fractions[i] << " ";
#endif
    }
    std::cerr << "sum of fractions is :" << sum_fractions << "\n";
    std::cerr << "Fractions of cloud in Both, GC and QC :";
#ifdef SAVE_COV_DATA
    resout << "\n";
#endif
    for (int i=0; i<3; ++i){
        cloud_fractions[i] = cloud_fractions[i]/sum_fractions;
        std::cerr << cloud_fractions[i] << " ";
#ifdef SAVE_COV_DATA
        resout << cloud_fractions[i] << " ";
#endif
#ifdef PRINT_NN_DATA
        fro << cloud_fractions[i] << " ";
#endif
    }
    std::cerr<<"\n";
#ifdef SAVE_COV_DATA
    resout << "\n";
    resout.close();
    data_generated = true;
#endif
#ifdef PRINT_NN_DATA
    fro << "\n";
    fro.close();
    for (int i=0; i<3; ++i){
        fout[i].close();
    }
    data_generated = true;
#endif
}

void CoveragePC::spin(){
    ros::Rate loop_rate(2);
    while (ros::ok()){
        ros::spinOnce();
        if (!data_generated){
            findCorrespondences();
            estimateCoverage();
        }
        loop_rate.sleep();
        //if (!continueLoop()) break;
        ROS_INFO(" ");
    }
}

bool continueLoop(){
    char x;
    std::cout << "Continue? ";
    std::cin >> x;
    if (x=='n'){
        return false;
    }else{
        return true;
    }
}

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud,
        pcl::KdTreeFLANN<Point>& kdtree, int K){
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        return pointIdxNKNSearch;
    }
    pointIdxNKNSearch.clear();
    return pointIdxNKNSearch;
}

void convertPoints(pcl::PointXYZRGB& p, Point& q){
    p.x = q.x;
    p.y = q.y;
    p.z = q.z;
}

void colorIt(pcl::PointXYZRGB& p, int color){
    uint8_t r=0,g=0,b=0;
    switch(color){
        case 0: r = 255; break;
        case 1: g = 255; break;
        case 2: b = 255; break;
        default : std::cerr<< "  What color?\n";
    }
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    p.rgb = *reinterpret_cast<float*>(&rgb);
}

tf::StampedTransform getTransform(std::string from_frame, std::string to_frame){
    //ROS_INFO("Finding transform from: %s to: %s",from_frame.c_str(), to_frame.c_str());
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

Point transformPoint(Point& p, tf::StampedTransform t){
    tf::Vector3 p_temp(p.x, p.y, p.z);
    tf::Vector3 tp = t.inverse() * p_temp;
    Point q;
    q.x = tp[0];
    q.y = tp[1];
    q.z = tp[2];
    q.rgb = p.rgb;
    return q;
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "coverage_pc");
    gt_name = "cube1";
    qd_name = "cube2";
    if (argc == 3){
        gt_name = argv[1];
        qd_name = argv[2];
    }
    CoveragePC test;
    test.spin();
    return 0;
}
