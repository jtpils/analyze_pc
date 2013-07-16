#include <createCubePCL/analyze_pc.h>
#include "dm_colors.hpp"
#include "utils.hpp"

#define WORLD_FRAME "/world"
//#define CORRESPONDENCE_ONLY
#define ERROR_LINES_DISPLAY
//#define VIEW_FPFH_HISTOGRAMS
#define SAVE_FPFH_HISTOGRAMS

std::string gt_name;
std::string qd_name;
bool found_kp;
bool found_fh;
static std::string names[10]={"bun000", "bun045", "bun090", "bun180", "bun270", "bun315", "chin", "ear_back", "top2", "top3"};
int pair_number;

AnalyzePC::AnalyzePC():
gt_cloud(new pcl::PointCloud<Point>),
qd_cloud(new pcl::PointCloud<Point>),
transformed_qd_cloud(new pcl::PointCloud<Point>),
keypoints_gt(new pcl::PointCloud<pcl::PointXYZI>),
keypoints_qd(new pcl::PointCloud<pcl::PointXYZI>),
fpfhs_gt(new pcl::PointCloud<pcl::FPFHSignature33>),
fpfhs_qd(new pcl::PointCloud<pcl::FPFHSignature33>)
{
    std::string gt_cloud_topic_name = "/"+gt_name+"/cloud";
    std::string qd_cloud_topic_name = "/"+qd_name+"/cloud";
    gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &AnalyzePC::gtCloudCb, this);
    qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &AnalyzePC::qdCloudCb, this);
    vis_pub = nh.advertise<visualization_msgs::Marker>("/cloud_vis_marker",1, this);
    kpg_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+gt_name+"/kp_cloud",1);
    kpq_pub = nh.advertise<sensor_msgs::PointCloud2>("/"+qd_name+"/kp_cloud",1);
    registered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>
        ("/"+qd_name+"/registered_cloud", 1);
    registered_kp_pub = nh.advertise<sensor_msgs::PointCloud2>
        ("/"+qd_name+"/registered_kp_cloud", 1);
    set_parameters_server = nh.advertiseService("/analyze_pc/set_parameters",
            &AnalyzePC::setParamCb, this);

    harris_radius = 0.1;
    nh.setParam("/analyze_pc/harris_radius", harris_radius);
    normal_estimation_radius = 0.2;
    nh.setParam("/analyze_pc/normal_estimation_radius", normal_estimation_radius);
    fpfh_estimation_radius = 0.4;
    nh.setParam("/analyze_pc/fpfh_estimation_radius", fpfh_estimation_radius);
    min_sample_distance = 0.1;
    nh.setParam("/analyze_pc/sacia/min_sample_distance", min_sample_distance);
    max_correspondence_distance = 0.1*0.1;
    nh.setParam("/analyze_pc/sacia/max_correspondence_distance",
            max_correspondence_distance);
    nr_iterations = 500;
    nh.setParam("/analyze_pc/sacia/nr_iterations", nr_iterations);

    feature_added_gt = false;
    feature_added_qd = false;
    min_fitness_score = FLT_MAX;
}

void AnalyzePC::gtCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *gt_cloud);
}

void AnalyzePC::qdCloudCb(const sensor_msgs::PointCloud2ConstPtr& input){
    pcl::fromROSMsg(*input, *qd_cloud);
}

void AnalyzePC::showKeyPoints(bool cache){
    sensor_msgs::PointCloud2 kp_pc;
    if (cache){
        if ((pcl::io::loadPCDFile<pcl::PointXYZI>("kp_"+gt_name+".pcd", *keypoints_gt) != -1) and (pcl::io::loadPCDFile<pcl::PointXYZI>("kp_"+qd_name+".pcd", *keypoints_qd) != -1)){
            pcl::toROSMsg(*keypoints_gt, kp_pc);
            kp_pc.header.frame_id=gt_cloud->header.frame_id;
            kpg_pub.publish(kp_pc);
            pcl::toROSMsg(*keypoints_qd, kp_pc);
            kp_pc.header.frame_id=qd_cloud->header.frame_id;
            kpq_pub.publish(kp_pc);
            found_kp = true;
            return;
        }else{
            ROS_ERROR("Cached files read fail :kp_gt.pcd or kp_qd.pcd");
        }
    }
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Finding the keypoints");
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp_gt;
    pcl::search::KdTree<Point>::Ptr tree_gt(new pcl::search::KdTree<Point>());
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp_qd;
    pcl::search::KdTree<Point>::Ptr tree_qd(new pcl::search::KdTree<Point>());

    hkp_gt.setRadius(harris_radius);
    hkp_gt.setSearchMethod(tree_gt);
    hkp_gt.setInputCloud(gt_cloud);
    hkp_gt.compute(*keypoints_gt);
    keypoints_gt->header.frame_id=gt_cloud->header.frame_id;
    pcl::toROSMsg(*keypoints_gt, kp_pc);
    kpg_pub.publish(kp_pc);
    pcl::io::savePCDFileASCII ("kp_"+gt_name+".pcd", *keypoints_gt);
    ROS_INFO("Found GT_CLOUD (%s) keypoints :%d", gt_name.c_str(), keypoints_gt->points.size());

    hkp_qd.setRadius(harris_radius);
    hkp_qd.setSearchMethod(tree_qd);
    hkp_qd.setInputCloud(qd_cloud);
    hkp_qd.compute(*keypoints_qd);
    keypoints_qd->header.frame_id=qd_cloud->header.frame_id;
    pcl::toROSMsg(*keypoints_qd, kp_pc);
    kpq_pub.publish(kp_pc);
    pcl::io::savePCDFileASCII ("kp_"+qd_name+".pcd", *keypoints_qd);
    ROS_INFO("Found QD_CLOUD (%s) keypoints :%d", qd_name.c_str(), keypoints_qd->points.size());
    found_kp = true;
}

void AnalyzePC::estimateFPFHFeatures(bool cache){
    if (cache){
        if (pcl::io::loadPCDFile<pcl::FPFHSignature33>((std::string)"fpfhs_"+gt_name+".pcd", *fpfhs_gt) != -1 and pcl::io::loadPCDFile<pcl::FPFHSignature33>((std::string)"fpfhs_"+qd_name+".pcd", *fpfhs_qd) != -1){
            found_fh = true;
            return;
        }else{
            ROS_ERROR("Cached files read fail :fpfhs_gt.pcd or fpfhs_qd.pcd");
        }
    }
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
    fpfh.compute(*fpfhs_gt);
    fpfhs_gt->width = fpfhs_gt->size();
    fpfhs_gt->height = 1;

    ROS_INFO("Found GT_CLOUD (%s) feature histogram :%d", gt_name.c_str(), fpfhs_gt->size());
#ifdef SAVE_FPFH_HISTOGRAMS
    pcl::io::savePCDFileASCII ("fpfhs_"+gt_name+".pcd", *fpfhs_gt);
#endif

#ifdef VIEW_FPFH_HISTOGRAMS
    if (!feature_added_gt){
        hist.addFeatureHistogram (*fpfhs_gt, 33 , "fpfh_dist_gt", 640, 200);
        feature_added_gt= true;
    }else{
        hist.updateFeatureHistogram (*fpfhs_gt, 33 , "fpfh_dist_gt");
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
    fpfh.compute(*fpfhs_qd);
    fpfhs_qd->width = fpfhs_qd->size();
    fpfhs_qd->height = 1;

    ROS_INFO("Found QD_CLOUD (%s) feature histogram :%d", qd_name.c_str(), fpfhs_qd->size());
#ifdef SAVE_FPFH_HISTOGRAMS
    pcl::io::savePCDFileASCII ("fpfhs_"+qd_name+".pcd", *fpfhs_qd);
#endif

#ifdef VIEW_FPFH_HISTOGRAMS
    if (!feature_added_qd){
        hist.addFeatureHistogram (*fpfhs_qd, 33 , "fpfh_dist_qd", 640, 200);
        feature_added_qd = true;
    }else{
        hist.updateFeatureHistogram (*fpfhs_qd, 33 , "fpfh_dist_qd");
    }
#endif
    found_fh = true;
}

void AnalyzePC::applyICP(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    pcl::PointCloud<Point> ransaced_source;
    pcl::KdTreeFLANN<Point> kdtree;
    sensor_msgs::PointCloud2 pc;
    float fitness_score;
    float min_error = FLT_MAX;
    float max_error = 0.0;
    float error = 0.0;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(qd_cloud);
    icp.setInputTarget(gt_cloud);
    ROS_INFO("");
    ROS_INFO("Applying ICP to get a finer transformation");

    //icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setMaximumIterations(nr_iterations);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.000001);

    icp.align(*transformed_qd_cloud, transformation_q_g);

    transformation_q_g = icp.getFinalTransformation();
    fitness_score = icp.getFitnessScore();
    ROS_INFO("Pointclouds aligned, fitness score (after ICP) is :%f", fitness_score);

    fitness_score = 0.0;
    error = 0.0;
    pcl::transformPointCloud(*(toPointXYZ(keypoints_qd)), ransaced_source, transformation_q_g);
    pcl::PointCloud<Point>::Ptr kp_gt_cl = toPointXYZ(keypoints_gt);
    kdtree.setInputCloud(kp_gt_cl);
    ransaced_source.width = ransaced_source.size();
    ransaced_source.height = 1;
    for (size_t i=0; i < ransaced_source.width; ++i){
        Point searchPoint = ransaced_source.points[i];
        int point_index = findNearestPointIndices(searchPoint, kp_gt_cl, kdtree, 1)[0];
        for (int j=0; j<33; ++j){
            float diff = fpfhs_gt->points[point_index].histogram[j]-fpfhs_qd->points[i].histogram[j];
            error += diff*diff;
        }
    }
    ROS_INFO("Keypoint Fitness Match (Euclidean) is %f",error/ransaced_source.size());
    fitness_score = 0.0;
    error = 0.0;
    kdtree.setInputCloud(gt_cloud);
    for (size_t i=0; i<transformed_qd_cloud->width; ++i){
        Point searchPoint = transformed_qd_cloud->points[i];
        int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree, 1)[0];
        Point resultPoint = gt_cloud->points[point_index];
        Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
        Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
        error = (p1-p2).squaredNorm();
        if (sqrt(error) < max_correspondence_distance){
            error = error/2.0;
        }else{
            error = max_correspondence_distance*(sqrt(error)-max_correspondence_distance/2.0);
        }
        if (error > max_error){
            max_error = error;
        }
        if (error < min_error){
            min_error = error;
        }
        fitness_score += fabs(error);
    }
    float avg_error = fitness_score/transformed_qd_cloud->width;
    ROS_INFO("Average error (huber fitness score) is %f",avg_error);
    if (avg_error <= min_fitness_score){
        pcl::io::savePCDFileASCII("registered_kp_"+gt_name+"_"+qd_name+".pcd", ransaced_source);
        pcl::toROSMsg(ransaced_source, pc);
        pc.header.frame_id = qd_cloud->header.frame_id;
        registered_kp_pub.publish(pc);

        pcl::io::savePCDFileASCII("registered_cloud_"+gt_name+"_"+qd_name+".pcd", *transformed_qd_cloud);
        pcl::toROSMsg(*transformed_qd_cloud, pc);
        pc.header.frame_id = gt_cloud->header.frame_id;
        registered_cloud_pub.publish(pc);
        min_fitness_score = avg_error;
    }
}

void AnalyzePC::applySACIA(){
    if (gt_cloud->points.size()==0 or qd_cloud->points.size()==0){
        ROS_ERROR("Not yet received point clouds");
        return;
    }
    ROS_INFO("Applying SAC-IA algorithm on the harris keypoint clouds using fpfh features");
    pcl::PointCloud<Point> ransaced_source;
    pcl::KdTreeFLANN<Point> kdtree;
    sensor_msgs::PointCloud2 pc;
    float fitness_score;
    float min_error = FLT_MAX;
    float max_error = 0.0;
    float error = 0.0;
    char x='y';
    sac_ia.setMinSampleDistance(min_sample_distance);
    //sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia.setMaximumIterations(nr_iterations);

    sac_ia.setInputCloud(toPointXYZ(keypoints_qd));
    sac_ia.setSourceFeatures(fpfhs_qd);
    sac_ia.setInputTarget(toPointXYZ(keypoints_gt));
    sac_ia.setTargetFeatures(fpfhs_gt);
    int count = 0;
    int max_count = 15;
    while (x!='n' and count<max_count){
        sac_ia.align(ransaced_source);
        fitness_score = sac_ia.getFitnessScore(max_correspondence_distance);
        transformation_q_g = sac_ia.getFinalTransformation();
        ROS_INFO("Pointclouds aligned, fitness score (with keypoints only) is :%f", fitness_score);
        ransaced_source.width = ransaced_source.size();
        ransaced_source.height = 1;

        pcl::PointCloud<Point>::Ptr kp_gt_cl = toPointXYZ(keypoints_gt);
        kdtree.setInputCloud(kp_gt_cl);
        error = 0.0;
        for (size_t i=0; i < ransaced_source.width; ++i){
            Point searchPoint = ransaced_source.points[i];
            int point_index = findNearestPointIndices(searchPoint, kp_gt_cl, kdtree, 1)[0];
            for (int j=0; j<33; ++j){
                float diff = fpfhs_gt->points[point_index].histogram[j]-fpfhs_qd->points[i].histogram[j];
                error += diff*diff;
            }
        }
        ROS_INFO("Keypoint Fitness Match (Euclidean) is %f",error/ransaced_source.size());

        pcl::transformPointCloud(*qd_cloud, *transformed_qd_cloud, transformation_q_g);
        kdtree.setInputCloud(gt_cloud);
        fitness_score = 0.0;
        for (size_t i=0; i<transformed_qd_cloud->width; ++i){
            Point searchPoint = transformed_qd_cloud->points[i];
            int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree, 1)[0];
            Point resultPoint = gt_cloud->points[point_index];
            Eigen::Vector4f p1 = Eigen::Vector4f (searchPoint.x, searchPoint.y, searchPoint.z, 0);
            Eigen::Vector4f p2 = Eigen::Vector4f (resultPoint.x, resultPoint.y, resultPoint.z, 0);
            error = (p1-p2).squaredNorm();
            if (sqrt(error) < max_correspondence_distance){
                error = error/2.0;
            }else{
                error = max_correspondence_distance*(sqrt(error)-max_correspondence_distance/2.0);
            }
            if (error > max_error){
                max_error = error;
            }
            if (error < min_error){
                min_error = error;
            }
            fitness_score += fabs(error);
        }
        float avg_error = fitness_score/transformed_qd_cloud->width;
        ROS_INFO("Average error (huber fitness score) is %f",avg_error);
        if (avg_error <= min_fitness_score){
            pcl::io::savePCDFileASCII("registered_kp_"+gt_name+"_"+qd_name+".pcd", ransaced_source);
            pcl::toROSMsg(ransaced_source, pc);
            pc.header.frame_id = qd_cloud->header.frame_id;
            registered_kp_pub.publish(pc);

            pcl::io::savePCDFileASCII("registered_cloud_"+gt_name+"_"+qd_name+".pcd", *transformed_qd_cloud);
            pcl::toROSMsg(*transformed_qd_cloud, pc);
            pc.header.frame_id = gt_cloud->header.frame_id;
            registered_cloud_pub.publish(pc);
            min_fitness_score = avg_error;
            std::cout << "Continue? :";
            x='y';
            //std::cin >> x;
            count = 0;
        }else{
            x='y';
            count++;
        }
    }
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
    marker.id = 0;
    marker.ns = "cube";
    marker.type = visualization_msgs::Marker::POINTS;
#ifdef ERROR_LINES_DISPLAY
    marker.type = visualization_msgs::Marker::LINE_LIST;
#endif
    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
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

    //pcl::transformPointCloud(*qd_cloud, *transformed_qd_cloud, transformation_q_g);
    transformed_qd_cloud = qd_cloud;

    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(gt_cloud);
    for (size_t i=0; i<transformed_qd_cloud->width; ++i){
        geometry_msgs::Point p;
        geometry_msgs::Point q;
        Point searchPoint = transformed_qd_cloud->points[i];
        p.x = searchPoint.x;
        p.y = searchPoint.y;
        p.z = searchPoint.z;
        marker.points.push_back(p);
        // Not assuming exact correspondence
        int point_index = findNearestPointIndices(searchPoint, gt_cloud, kdtree, 1)[0];
        q.x = gt_cloud->points[point_index].x;
        q.y = gt_cloud->points[point_index].y;
        q.z = gt_cloud->points[point_index].z;
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
        if (sqrt(error) < max_correspondence_distance){
            //error = error/2.0;
        }else{
            //error = max_correspondence_distance*(sqrt(error)-max_correspondence_distance/2.0);
        }
        if (error > max_error){
            max_error = error;
        }
        if (error < min_error){
            min_error = error;
        }
        error_data.push_back(error);
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
    ROS_INFO("Average error (huber fitness score) is %f",avg_error);
    if (avg_error <= min_fitness_score){
        sensor_msgs::PointCloud2 pc;
        pcl::toROSMsg(*transformed_qd_cloud, pc);
        pc.header.frame_id = gt_cloud->header.frame_id;
        registered_cloud_pub.publish(pc);
        min_fitness_score = avg_error;
        vis_pub.publish(marker);
    }
}

bool AnalyzePC::setParamCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    nh.getParam("/analyze_pc/harris_radius", harris_radius);
    nh.getParam("/analyze_pc/normal_estimation_radius", normal_estimation_radius);
    nh.getParam("/analyze_pc/fpfh_estimation_radius", fpfh_estimation_radius);
    nh.setParam("/analyze_pc/sacia/min_sample_distance", min_sample_distance);
    nh.setParam("/analyze_pc/sacia/max_correspondence_distance",
            max_correspondence_distance);
    nh.setParam("/analyze_pc/sacia/nr_iterations", nr_iterations);
    return true;
}

pcl::PointCloud<Point>::Ptr AnalyzePC::getCloud(std::string base_name){
    std::string package_path = ros::package::getPath("createCubePCL");
    pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
    pcl::io::loadPCDFile(package_path+"/data/"+base_name+"_UnStructured.pcd", *cloud);
    cloud->header.frame_id = "laser_"+base_name;
    return cloud;
}

void AnalyzePC::spin(){
    ROS_INFO("Next Pair :%s , %s", gt_name.c_str(), qd_name.c_str());
    ros::Rate loop_rate(10);
    found_kp = false;
    found_fh = false;
    while(ros::ok()){
        ros::spinOnce();
        //showKeyPoints(found_kp);
        //estimateFPFHFeatures(found_fh);
        //gt_cloud = getCloud(gt_name);
        //qd_cloud = getCloud(qd_name);
        /*
        showKeyPoints(true);
        estimateFPFHFeatures(true);
        applySACIA();
        applyICP();
        char x;
        std::cout << "Continue?";
        */
        //std::cin >> x;

        /*
        if (found_kp and found_fh and pair_number+3<10){
            pair_number+=1;
            gt_name = names[pair_number];
            qd_name = names[(pair_number+1)];
            ROS_INFO("Next Pair :%s , %s", gt_name.c_str(), qd_name.c_str());
            gt_cloud = getCloud(gt_name);
            qd_cloud = getCloud(qd_name);
            std::string gt_cloud_topic_name = "/"+gt_name+"/cloud";
            std::string qd_cloud_topic_name = "/"+qd_name+"/cloud";
            gt_cloud_sub.shutdown();
            qd_cloud_sub.shutdown();
            gt_cloud_sub = nh.subscribe(gt_cloud_topic_name, 1, &AnalyzePC::gtCloudCb, this);
            qd_cloud_sub = nh.subscribe(qd_cloud_topic_name, 1, &AnalyzePC::qdCloudCb, this);
            found_kp = false;
            found_fh = false;
        }
        */
        visualizeError();
#ifdef VIEW_FPFH_HISTOGRAMS
        hist.spinOnce(10);
#endif
        ROS_INFO(" ");
        loop_rate.sleep();
    }
}

int main (int argc, char ** argv){
    ros::init (argc, argv, "analyze_pc");
    pair_number = 0;
    gt_name = names[pair_number];
    qd_name = names[pair_number+2];
    if (argc == 3){
        gt_name = argv[1];
        qd_name = argv[2];
    }
    AnalyzePC test;
    test.spin();
    return 0;
}
