
//////// std includes ////////
#include <stdio.h>
#include <omp.h>

/////// ROS ///////
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <laser_assembler/AssembleScans.h>
#include <tf/transform_datatypes.h>
#include <ros/publisher.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/package.h>

/////// PCL ///////
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ros/conversions.h>

#include <pcl/features/normal_3d_omp.h>
// #include <pcl/features/our_cvfh.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// #include <pcl/recognition/cg/correspondence_grouping.h>
// #include <pcl/recognition/cg/geometric_consistency.h>
// #include <pcl/recognition/cg/hough_3d.h>
// #include <pcl/recognition/cg/geometric_consistency.h>

// #include <pcl/recognition/hv/hv_papazov.h>
// #include <pcl/recognition/hv/hv_go.h>
// #include <pcl/recognition/hv/greedy_verification.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
// #include <pcl/registration/gicp.h> //generalized icp extension
// #include <pcl/registration/icp_nl.h> //Levenberg-Marquardt icp 

/////// TROOPER includes ///////
#include <trooper_msgs/DetectedObjectList.h>

// #define DEBUG //debugging output

typedef pcl::PointXYZ PointT;

struct Model
{
    int id; ///<model view id ... not really used anywhere but useful for debugging maybe
    pcl::PointCloud<PointT>::Ptr cloud; ///<model cloud from sensor point of view
    pcl::PointCloud<PointT>::Ptr cloud_sampled; ///<downsampled cloud (used for first phase of pose estimate)
    geometry_msgs::PosePtr pose;///<pose relative to robot during recording
};

class ObjDetector
{
public:
    ///Default constructor
    ObjDetector();
    ///Default destructor
    ~ObjDetector(){/*empty*/}
    
    ///\brief initialize object detector by reading parameters and loading models
    ///\return false if loading of models failed or parameters wrong/missing, true otherwise
    bool initialize();
    
    ///\brief load models from database
    ///\param db_file[in] database file storing point clouds and associated poses
    ///\return true if loading was successful
    bool loadModels(const std::string &db_file);
    
    ///\brief try to match cluster
    ///\param cluster point cloud of cluster
    ///\return view id if found, -1 otherwise
    void detect(const pcl::PointCloud< PointT >::ConstPtr& cluster, int *matchedModelID, Eigen::Matrix4f *matchedTransform, Eigen::Matrix4f *matchedTransform2, const pcl::visualization::PCLVisualizer::Ptr &visu);
    
    ///\brief Returns model with id "id"
    void getModel(int id, Model *model);
    
    ///\brief Set max icp iterations
    void setMaxIter(int maxIter) {max_iter_ = maxIter;}
    
    ///\brief Set threshold for valid matches
    void setScoreThreshold(double thresh){score_thresh_ = thresh;}
private:
    ros::ServiceClient client_;
    ros::Publisher pub_;
    ros::Subscriber trigger_sub_;
    
    /// stores the object type the detector is detecting
    std::string obj_type_;
    /// models for all the views of the object
    std::vector<Model> models_;
    /// cluster bounds for pre-filtering (min height, min width, max height, max width) 
    std::vector<double> cluster_bounds_; //pre-filter constraints for clusters to discard irrelevant clusters and speed up detection 
                                         //... get rid of this maybe when switching to our cvfh? Could still be useful for speed up purposes though.
                                         //We're not really expecting a lot of clutter in the two VRC scenarios though so maybe useless
    /// leaf size for voxel grid sampling of model and clusters
    double sample_size_;
    /// max distance tolerance during clustering ... needs to be larger for polaris due to large gaps when viewed from behind
    double cluster_tolerance_; 
    double score_thresh_; ///< threshold for selecting a valid match 
                          //TODO: add 2 thresholds for rough and fine match?
    int max_iter_; ///< max ICP iterations for model refinement
    bool on_table_; ///< flag whether object can be on a table ... mainly for optimization right now
    
    
    ///callback for receiving detection commands
    void detectCallback(const std_msgs::StringConstPtr &cmd);
    
    ///extract clusters from scene assuming ground plane
    void extractClusters(const pcl::PointCloud<PointT>::ConstPtr &scene, std::vector<pcl::PointCloud<PointT>::Ptr> *clusters, const pcl::visualization::PCLVisualizer::Ptr &visu);
};


///comparator for pair of double and int
bool pairComparator( const std::pair<double,int>& l, const std::pair<double,int>& r)
{ 
    return l.first < r.first; 
}


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "obj_detector");

    
    //TODO:FIXME:remove this once done developing (sets logging level for this node to Debug)
    log4cxx::LoggerPtr roslogger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    roslogger->getLevel();
    roslogger->setLevel(ros::console::levels::Debug);

    ObjDetector detector;
    
    if(!detector.initialize())
    {
        ROS_FATAL("Object detector initialization failed. Shutting down node.");
        return 1;
    }

    ros::spin();
    
    return 0;
}

ObjDetector::ObjDetector()
:obj_type_(""), cluster_bounds_(4,0.0), sample_size_(0.05), cluster_tolerance_(0.05), score_thresh_(0.07), max_iter_(100), on_table_(false)
{

}


bool ObjDetector::initialize()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    
    //max iterations for ICP
    nh_local.getParam("iter", max_iter_);
    
    //score threshold for matching
    score_thresh_ = 0.07;//TODO: add 2 thresholds for rough and fine match?
    
    //load db
    std::string db_file = "";
    if(!nh_local.getParam("db", db_file))
    {
        ROS_ERROR("No database file specified!");
        return false;
    }

    if(!loadModels(db_file))
    {
        ROS_FATAL("AP: Failed loading model database \"%s\"!\n", db_file.c_str());
        return 1;
    }    
    
    
    //subscribe to spinning lidar cloud service
    ros::service::waitForService("/assemble_scans");
    client_ = nh.serviceClient<laser_assembler::AssembleScans>("/assemble_scans");
    
    //advertise object detections
    std::string det_topic = nh.resolveName("/ap/detected_objects");
    pub_ = nh.advertise<trooper_msgs::DetectedObjectList>(det_topic, 1, true); //TODO: latch or not latch?
    
    
    //subscribe to detection trigger topic
    std::string trigger_topic = nh.resolveName("/ap/run_object_detection");
    trigger_sub_ = nh.subscribe<std_msgs::String>(trigger_topic, 1, &ObjDetector::detectCallback, this);
    ROS_INFO("subscribed to %s", trigger_sub_.getTopic().c_str());
    return true;
}



void ObjDetector::detectCallback(const std_msgs::StringConstPtr &cmd)
{
    //TODO:for now we ignore the content of the cmd message
    ROS_INFO("AP: object detection triggered (msg \"%s\")\n", cmd->data.c_str());
    
    //////////////////
    // 0. get scene //
    //////////////////
    laser_assembler::AssembleScans srv;
    srv.request.begin = ros::Time(0);
    srv.request.end   = ros::Time::now();
    if (client_.call(srv))
    {
        ROS_DEBUG("AP: Got scene with %lu points.\n", srv.response.cloud.points.size());
    }
    else
    {
        ROS_ERROR("AP: Service call failed!\n");
        return;
    }
    
    //create point cloud from msg
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud);
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud, *scene);
    
    //visualize scene
    pcl::visualization::PCLVisualizer::Ptr vis;
#ifdef DEBUG
    vis.reset(new pcl::visualization::PCLVisualizer("TR00P3R -- Debug"));
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> scene_handler(scene, "x");//100, 100, 200);
    vis->addPointCloud(scene, scene_handler, "scene");
    vis->addCoordinateSystem();
    vis->setCameraPosition(-10,2,4,10,2,0,0,0,1);
    vis->spin();
#endif
    
    
    ////////////////////////////////////
    // 1. extract clusters from scene //
    ////////////////////////////////////
    ROS_DEBUG("AP: clustering point cloud.\n");
    std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
    extractClusters(scene, &clusters, vis);
    
    //visualize clusters
#ifdef DEBUG
//     vis->removePointCloud("scene");
//     for(int i = 0; i < clusters.size(); ++i)
//     {
//         std::stringstream ss;
//         ss << "cluster_raw_" << i;
// //         pcl::visualization::PointCloudColorHandlerCustom<PointT> cluster_handler(clusters[i], 0, 0, 255.0);
//         pcl::visualization::PointCloudColorHandlerRandom<PointT> cluster_handler(clusters[i]);
//         vis->addPointCloud(clusters[i], cluster_handler, ss.str());
//         vis->spinOnce();
//     }
#endif
    
    ////////////////////////////////////////////////////////////////
    // 2. perform simple filtering to discard impossible clusters //
    //    TODO: remove when switching to our cfvh?                //
    ////////////////////////////////////////////////////////////////
    std::vector<pcl::PointCloud<PointT>::Ptr> cluster_candidates;
    for(int i = 0; i < clusters.size(); ++i)
    {
        pcl::PointCloud<PointT>::Ptr cluster = clusters[i];
        
        PointT minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);
        
        float height = maxPt.z-minPt.z;
        float width = std::max(maxPt.x-minPt.x, maxPt.y-minPt.y);
        //if(height <= OBJ_MAX_HEIGHT && height >= OBJ_MIN_HEIGHT &&  width <= OBJ_MAX_WIDTH && width >= OBJ_MIN_WIDTH) //(min height, min width, max height, max width) 
        if(width >= cluster_bounds_[0] && width <= cluster_bounds_[1] && height >= cluster_bounds_[2] && height <= cluster_bounds_[3]) //min width, max width, min height, max height
        {
            cluster_candidates.push_back(cluster);
//             ROS_INFO("cluster candidate added (h:%f,w:%f)\n", height, width);
        }
        else
        {
//             ROS_ERROR("%f<=%f<=%f or %f<=%f<=%f invalid\n", OBJ_MIN_HEIGHT, height, OBJ_MAX_HEIGHT, OBJ_MIN_WIDTH, width, OBJ_MAX_WIDTH);
        }
    }
    
    //visualize cluster candidates
#ifdef DEBUG
    vis->removePointCloud("scene");
    for(int i = 0; i < cluster_candidates.size(); ++i)
    {
        std::stringstream ss;
        ss << "cluster_" << i;
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cluster_handler(cluster_candidates[i], 0, 100, 255.0*double(i)/cluster_candidates.size());
        
        vis->addPointCloud(cluster_candidates[i], cluster_handler, ss.str());
        vis->spinOnce();
    }
    vis->spin();
#endif
    
    ROS_DEBUG("%d/%d clusters are candidates\n", cluster_candidates.size(), clusters.size());
    
    ////////////////////////////////////////////////
    // 3. run detection on each cluster candidate //
    ////////////////////////////////////////////////
    ROS_DEBUG("AP: starting detection ...\n");
    
#ifdef DEBUG
    vis->addPointCloud(scene, scene_handler, "scene");
#endif
    
    std::vector<Model> detections;
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > detection_transforms;
    //TODO: not sure whether using parallelization here makes a lot of sense given that the detector uses a thread per viewpoint as well
    //...when switching to our cvfh we can definitely parallelize here though as the inner detector won't need threads then I think
    pcl::ScopeTime t_all;
    #pragma omp parallel for 
    for(int i = 0; i < cluster_candidates.size(); ++i)
    {
        ROS_DEBUG("AP: running detection on cluster %d\n", i);
        
        Eigen::Matrix4f detection_transform, detection_transform2;
        int detection_id = -1;
        pcl::ScopeTime t;
        detect(cluster_candidates[i], &detection_id, &detection_transform, &detection_transform2, vis);
        ROS_DEBUG("Detection for cluster %d took %fs\n", i, t.getTimeSeconds());
        
        if(detection_id != -1)
        {
            Model m;
            getModel(detection_id, &m);

//             #pragma omp critical
            {
                detections.push_back(m);
                detection_transforms.push_back(detection_transform);
            
            
                //visualize 3D model of detection
#ifdef DEBUG
                vtkSmartPointer<vtkTransform> vtkTrans(vtkTransform::New());
                const double vtkMat[16] = {(detection_transform)(0,0), (detection_transform)(0,1),(detection_transform)(0,2),(detection_transform)(0,3),
                                        (detection_transform)(1,0), (detection_transform)(1,1),(detection_transform)(1,2),(detection_transform)(1,3),
                                        (detection_transform)(2,0), (detection_transform)(2,1),(detection_transform)(2,2),(detection_transform)(2,3),
                                        (detection_transform)(3,0), (detection_transform)(3,1),(detection_transform)(3,2),(detection_transform)(3,3)};
                vtkTrans->SetMatrix(vtkMat);
                std::stringstream visMatchName;
                static int visMatchedModelCounter = 0;
                visMatchName << "Polaris_Ranger_" << visMatchedModelCounter++;
                vis->addModelFromPLYFile("/home/thomas/TROOPER/DATA/VRC/models/polaris.ply", vtkTrans, visMatchName.str());
                
//                 pcl::PointCloud<PointT>::Ptr modelCloudMatched(new pcl::PointCloud<PointT>);
//                 pcl::transformPointCloud(*m.cloud, *modelCloudMatched, detection_transform2);
//                 pcl::visualization::PointCloudColorHandlerCustom<PointT> match_cloud_handler(scene, 255, 0, 0);
//                 vis->addPointCloud(modelCloudMatched, match_cloud_handler, visMatchName.str()+"_cloud");
                vis->addText3D(visMatchName.str(), PointT(detection_transform(0,3),detection_transform(1,3),detection_transform(2,3)+2.5), 0.08, 1.0, 1.0, 1.0, visMatchName.str()+"_text");
                vis->spinOnce();
#endif
            }
        }
    }
    ROS_DEBUG("Detection for all clusters took %fs\n", t_all.getTimeSeconds());
    
    ////////////////////////
    // 4. publish, yay :) //
    ////////////////////////
    trooper_msgs::DetectedObjectList objects;
    for(int i = 0; i < detections.size(); ++i)
    {
        trooper_msgs::DetectedObject obj;
        obj.header.frame_id = "map"; //FIXME: correct frame?
        obj.header.stamp = srv.request.end;
        std::stringstream ss;
        ss << "ap_" << i;
        obj.id = ss.str();
        obj.object_ref = obj_type_;
        obj.region3d.center.x = detection_transforms[i](0,3);
        obj.region3d.center.y = detection_transforms[i](1,3); 
        obj.region3d.center.z = detection_transforms[i](2,3); 
        
        Eigen::AngleAxisf angax;
        angax = detection_transforms[i].topLeftCorner<3,3>();
        obj.region3d.axis.x = angax.axis()(0); 
        obj.region3d.axis.y = angax.axis()(1);
        obj.region3d.axis.z = angax.axis()(2); 
        obj.region3d.angle = angax.angle(); 
        
        objects.detected_objects.push_back(obj);
    }

    pub_.publish(objects);
    
    ROS_INFO("AP: published object list containing %d objects\n", objects.detected_objects.size());
    
#ifdef DEBUG
    vis->spin();
//     ROS_ERROR("VISUALIZER IS DEAD!!!!\n");
//     vis->close();
// //     vis.reset();
//     ROS_ERROR("CLOSED?\n");
#endif    
}

void ObjDetector::extractClusters(const pcl::PointCloud< PointT >::ConstPtr& scene, std::vector< boost::shared_ptr< pcl::PointCloud< PointT > > >* clusters, const pcl::visualization::PCLVisualizer::Ptr &visu)
{
//TODO: removed plane extraction and replaced with faster but simpler passthrough filter 
//given that we project the cloud into map frame already anyway and only have a ground plane at z=~0
//Once we have the proper terrain fusion in place we don't need any of this anymore and don't depend on flat terrain
//(although we will always have at least mostly flat terrain below objects in the VRC anyway)
// //     /////////////////////////////////////
// //     // #1. filter and estimate normals //
// //     /////////////////////////////////////
// //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// //     pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEst;
// //     normalEst.setSearchMethod(tree);
// //     normalEst.setInputCloud(cloud_filtered);
// //     normalEst.setKSearch(25);
// //     normalEst.compute(*cloud_normals);
// //     
// //     //     vis.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, cloud_normals, 10, 0.02, "normals");
// //     
// //     
// //     ///////////////////////////
// //     // #2. find ground plane //
// //     ///////////////////////////
// //     
// //     //find biggest plane (=ground plane)
// //     pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
// //     pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
// //     
// //     pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sac;
// //     sac.setOptimizeCoefficients(true);
// //     sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
// //     sac.setAxis(Eigen::Vector3f(0,0,1)); //look for plane normal to z axis
// //     sac.setNormalDistanceWeight(0.1);
// //     sac.setMethodType(pcl::SAC_RANSAC);
// //     sac.setMaxIterations(100);
// //     sac.setDistanceThreshold(0.05); //10cm threshold - required for segmenting the ground at ~20m due to noise, can use a lower threshold for manipulation task
// //     sac.setInputCloud(cloud_filtered);
// //     sac.setInputNormals(cloud_normals);
// //     sac.segment(*inliers_plane, *coefficients_plane);
// //     std::cout << "plane coefficients: " << *coefficients_plane << std::endl;
// //     std::cout << "inliers: " << inliers_plane->indices.size() << std::endl;
// //     
// //     //remove plane from point cloud
// //     extractor.setInputCloud(cloud_filtered);
// //     extractor.setIndices(inliers_plane);
// //     extractor.setNegative(true);
// //     
// //     pcl::PointCloud<PointT>::Ptr cloud_segmented(new pcl::PointCloud<PointT>);
// //     extractor.filter(*cloud_segmented);
    
    ////////////////////////////////////////////////////
    // #0. remove points too far away, mainly needed  //
    // to make voxel filter work for small leaf sizes //
    ////////////////////////////////////////////////////
    const float X_MIN = 0.0, X_MAX = 20.0;
    pcl::PassThrough<PointT> scene_pass_filter_x;
    scene_pass_filter_x.setFilterFieldName("x");
    scene_pass_filter_x.setFilterLimits(X_MIN, X_MAX);
    scene_pass_filter_x.setKeepOrganized(false); //should remove NaN and Inf values
    scene_pass_filter_x.setInputCloud(scene);
    pcl::PointCloud<PointT>::Ptr scene_filtered_x(new pcl::PointCloud<PointT>);
    scene_pass_filter_x.filter(*scene_filtered_x);
    
#ifdef DEBUG
    visu->removeAllPointClouds();
    visu->addPointCloud(scene_filtered_x);
    visu->spin();
#endif
    
    const float Y_MIN = -10, Y_MAX = 10.0; //we have a lower angular resolution to the side
    pcl::PassThrough<PointT> scene_pass_filter_y;
    scene_pass_filter_y.setFilterFieldName("y");
    scene_pass_filter_y.setFilterLimits(Y_MIN, 10);
    scene_pass_filter_y.setKeepOrganized(false); //should remove NaN and Inf values
    scene_pass_filter_y.setInputCloud(scene_filtered_x);
    pcl::PointCloud<PointT>::Ptr scene_filtered_y(new pcl::PointCloud<PointT>);
    scene_pass_filter_y.filter(*scene_filtered_y);
    
#ifdef DEBUG
    visu->removeAllPointClouds();
    visu->addPointCloud(scene_filtered_y);
    visu->spin();
#endif
    
    //////////////////////////////////////////////
    // #1. remove ground plane and filter cloud //
    //////////////////////////////////////////////
    //remove plane from point cloud and points too high (e.g. the "sky" currently captured by gazebo)
    const float Z_MIN = 0.07, Z_MAX = 2.5; //FIXME: coordinate system might be different
    pcl::PassThrough<PointT> scene_pass_filter;
    scene_pass_filter.setFilterFieldName("z");
    scene_pass_filter.setFilterLimits(Z_MIN, Z_MAX);
    scene_pass_filter.setKeepOrganized(false); //should remove NaN and Inf values
    scene_pass_filter.setInputCloud(scene_filtered_y);
    
    pcl::PointCloud<PointT>::Ptr scene_filtered(new pcl::PointCloud<PointT>);
    scene_pass_filter.filter(*scene_filtered);
    
#ifdef DEBUG
    visu->removeAllPointClouds();
    visu->addPointCloud(scene_filtered);
    visu->spin();
#endif
    
    ROS_DEBUG("after pass filter: %d", scene_filtered->size());
    
    pcl::PointCloud<PointT>::Ptr scene_filtered_sampled(new pcl::PointCloud<PointT>);
    if(sample_size_ >= 0.01)
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setLeafSize(sample_size_, sample_size_, sample_size_);
        vg.setInputCloud(scene_filtered);
        vg.filter(*scene_filtered_sampled);
        
        ROS_INFO("after voxel filter: %d", scene_filtered_sampled->size());
    }
    else
    {
        ROS_DEBUG("voxel sample size of %f too small, skipping sampling step");
        scene_filtered_sampled = scene_filtered;
    }
    

    //if we're performing tabletop manipulation we also look for a table
    //TODO: more generic approach
    if(on_table_)
    {
        ROS_DEBUG("REMOVING PLANE!");
        /////////////////////////////////
        // filter and estimate normals //
        /////////////////////////////////
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEst;
        normalEst.setSearchMethod(tree);
        normalEst.setInputCloud(scene_filtered_sampled);
        normalEst.setKSearch(25);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        normalEst.compute(*cloud_normals);

        ////////////////
        // find table //
        ////////////////
        
        //find biggest plane (=ground plane)
        pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
        
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sac;
        sac.setOptimizeCoefficients(true);
        sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
        sac.setAxis(Eigen::Vector3f(0,0,1)); //look for plane normal to z axis
        sac.setNormalDistanceWeight(0.1);
        sac.setMethodType(pcl::SAC_RANSAC);
        sac.setMaxIterations(100);
        sac.setDistanceThreshold(0.05); //10cm threshold - required for segmenting the ground at ~20m due to noise, can use a lower threshold for manipulation task
        sac.setInputCloud(scene_filtered_sampled);
        sac.setInputNormals(cloud_normals);
        sac.setEpsAngle(0.04);
        sac.segment(*inliers_plane, *coefficients_plane);
        ROS_DEBUG_STREAM("plane coefficients: " << *coefficients_plane);
        ROS_DEBUG_STREAM("inliers: " << inliers_plane->indices.size());
        
        //remove plane from point cloud
        pcl::ExtractIndices<PointT> extractor;
        extractor.setInputCloud(scene_filtered_sampled);
        extractor.setIndices(inliers_plane);
        extractor.setNegative(true);
        
        pcl::PointCloud<PointT>::Ptr cloud_segmented(new pcl::PointCloud<PointT>);
        extractor.filter(*cloud_segmented);
        scene_filtered_sampled = cloud_segmented;
    }
    
    ///////////////////////
    // #2. find clusters //
    ///////////////////////
#ifdef DEBUG
    visu->removeAllPointClouds();
    visu->addPointCloud(scene_filtered_sampled);
    visu->spin();
#endif
    
    //extract clusters
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(scene_filtered);    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_); //25cm
    ec.setMinClusterSize(50); //need to go low for standpipe
    ec.setMaxClusterSize(10000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(scene_filtered_sampled);
    ec.extract(cluster_indices);
    
    ROS_DEBUG("AP: %u clusters found\n", cluster_indices.size());
    
    int j = 0;
    for(int i = 0; i < cluster_indices.size(); ++i)
    {
        ROS_DEBUG("AP: cluster %d has %u points\n", i, cluster_indices[i].indices.size()); 
        
        //extract cluster from point cloud
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        
        //TODO: i don't like this loopy loop, but index extractor didn't do the job :(
        for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); ++pit)
            cloud_cluster->points.push_back(scene_filtered_sampled->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true; //is_dense means there's no NaN/inf values, it's different from "organized"
        
        clusters->push_back(cloud_cluster);
        
        //display cluster
#ifdef DEBUG
        std::stringstream cluster_name; 
        cluster_name << "cluster_" << i;
        
        pcl::visualization::PointCloudColorHandlerRandom<PointT> random_handler(cloud_cluster);
        visu->addPointCloud<PointT>(cloud_cluster, random_handler, cluster_name.str());
#endif
    }
}

void ObjDetector::detect(const pcl::PointCloud< PointT >::ConstPtr& cluster, int* matchedModelID, Eigen::Matrix4f* matchedTransform, Eigen::Matrix4f* matchedTransform2, const pcl::visualization::PCLVisualizer::Ptr &visu)
{
    //run thread for each model to speed up process - correct model should return after ~0.5s 
    //TODO: optimize, use our-cvfh to prune possible models, try to stop other threads once one of them found a match
    
    std::vector<std::pair<double,int> > model_scores(models_.size()); //store model score with index for sorting
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_transforms(models_.size());

    //downsample cluster
    pcl::PointCloud<PointT>::Ptr cluster_sampled(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(sample_size_, sample_size_, sample_size_);
    vg.setInputCloud(cluster);
    vg.filter(*cluster_sampled);
    ROS_INFO("cluster has %d points after sampling (%d before)\n", cluster_sampled->size(), cluster->size());

    int numThreads = models_.size(); //there's only up to 8 views per model at the moment so this runs 1 thread per view
    int schedule_chunk_size = 1;//round(models_.size()/8);
    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, schedule_chunk_size)
    for(int modelIdx = 0; modelIdx < models_.size(); ++modelIdx)
    {
        pcl::PointCloud<PointT>::Ptr model = models_[modelIdx].cloud_sampled;
//         ROS_INFO("T%d:__3.5\n", omp_get_thread_num());
        
        ///////////////////////////////////////
        // 3. Shift clouds based on centroid //
        ///////////////////////////////////////
        //TODO: maybe add eigen vectors (orientation)?
        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster_sampled, centCluster);
        Eigen::Vector4f initialTransform = centCluster - centModel;

        pcl::PointCloud<PointT>::Ptr model_transformed(new pcl::PointCloud<PointT>);
        Eigen::Quaternionf rot(1.0, 0.0, 0.0, 0.0); //TODO: maybe add eigen vectors (orientation)?

        pcl::transformPointCloud(*model, *model_transformed, Eigen::Vector3f(initialTransform[0],initialTransform[1],initialTransform[2]), rot);

        ////////////////////////
        // 4. register clouds //
        ////////////////////////
        pcl::PointCloud<PointT>::Ptr modelRegistered(new pcl::PointCloud<PointT>);
        
        pcl::ScopeTime t("ICP...");
//         ROS_INFO("T%d:__6\n", omp_get_thread_num());
        pcl::Registration<PointT, PointT>::Ptr registration (new pcl::IterativeClosestPoint<PointT, PointT>);
        //         pcl::Registration<PointT, PointT>::Ptr registration(new pcl::IterativeClosestPointNonLinear<PointT, PointT>);
//         pcl::Registration<PointT, PointT>::Ptr registration(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>);
        registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster_sampled);
        registration->setMaxCorrespondenceDistance(0.25);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(max_iter_);
        
        ROS_INFO("Input to ICP: inputSource: %dpts, inputTarget: %dpts\n", model_transformed->size(), cluster_sampled->size());

        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();

        Eigen::Matrix4f initT;
        initT << 1, 0, 0, initialTransform[0],
                0, 1, 0, initialTransform[1],
                0, 0, 1, initialTransform[2],
                0, 0, 0, 1;
        
        ROS_INFO("score: %f\n", registration->getFitnessScore());
        model_scores[modelIdx] = std::make_pair<double,int>(registration->getFitnessScore(), modelIdx);
        model_transforms[modelIdx] = transformation_matrix * initT;
        
    }

    //sort model scores (lower score = better)
    std::sort(model_scores.begin(), model_scores.end(), pairComparator);
        
    if(model_scores[0].first < score_thresh_)
    {
        *matchedModelID = model_scores[0].second;

        
        //perform additional refinement on final model
        //TODO: remove? not needed so far actually, maybe when switching to our cvfh
//         pcl::PointCloud<PointT>::Ptr model_aligned(new pcl::PointCloud<PointT>);
//         pcl::transformPointCloud(*models_[*matchedModelID].cloud, *model_aligned, model_transforms[*matchedModelID]);

//         pcl::ScopeTime t("ICP2 ...");
//         pcl::Registration<PointT, PointT>::Ptr registration (new pcl::IterativeClosestPoint<PointT, PointT>);
//         registration->setInputSource(model_aligned);
//         registration->setInputTarget(cluster);
//         registration->setMaxCorrespondenceDistance(0.01);
//         registration->setRANSACOutlierRejectionThreshold(0.01);
//         registration->setTransformationEpsilon(0.000001);
//         registration->setMaximumIterations(40); //FIXME: make variable?
        
//         pcl::PointCloud<PointT>::Ptr modelRefined(new pcl::PointCloud<PointT>);
//         registration->align(*modelRefined);
        
//         Eigen::Matrix4f refined_transform = registration->getFinalTransformation();
//         std::cout << "REFINED MATRIX: " << refined_transform << std::endl;
        
        //calculate final transformation
        geometry_msgs::PosePtr mp = models_[*matchedModelID].pose;
        Eigen::Affine3f tr;
        tr = Eigen::Translation3f(mp->position.x, mp->position.y, mp->position.z) 
             * Eigen::Quaternionf(mp->orientation.w, mp->orientation.x, mp->orientation.y, mp->orientation.z);
        Eigen::Matrix4f initPose(tr.data()); //FIXME: does this actually work?
        *matchedTransform = /*refined_transform **/ model_transforms[*matchedModelID] * initPose;
        *matchedTransform2 = /*refined_transform **/ model_transforms[*matchedModelID];
        
        ROS_DEBUG("AP: Best match: view %d with score of %f\n", model_scores[0].second, model_scores[0].first);
        ROS_DEBUG_STREAM("AP: Transformation matrix: \n" << *matchedTransform);
    }
    else
    {
        ROS_DEBUG("AP: No matches found for cluster (might be a good thing!)\n");
    }
}

void ObjDetector::getModel(int id, Model* model)
{
    if(id>=0 && id < models_.size())
    {
        *model = models_[id];
    }
    else
    {
        *model = Model();
        ROS_ERROR("AP: no model with id %d! (%d models in db)\n", id, models_.size());
    }
}

bool ObjDetector::loadModels(const std::string& db_file)
{
    std::ifstream file;
    file.open(db_file.c_str());
    
    if(!file.is_open())
    {
        ROS_ERROR("File \"%s\" not found!\n", db_file.c_str());
        return false;
    }

    std::string db_path = ros::package::getPath("trooper_obj_recognition")+"/models/";
    
    
    enum LINE_DESCR {TYPE = 0, BOUNDS = 1, SAMPLESIZE = 2, CLUSTER_TOLERANCE = 3, ON_TABLE = 4, MODEL_VIEW_BEGIN};
    
    int lineNum = 0;
    while(!file.eof())
    {
        std::string line;
        getline(file, line);
        
        if(file.eof())
            break;
        
        std::stringstream lineParse(line);
        if(lineNum == TYPE) //first line is model type
        {
            lineParse >> obj_type_;
            ROS_INFO("AP: loading object type %s", obj_type_.c_str());
        }
        else if(lineNum == BOUNDS) //second line specifies min max bounds for clusters //TODO: maybe remove this when switching to our cvfh, but probably still useful
        {
            lineParse >> cluster_bounds_[0] >> cluster_bounds_[1] >> cluster_bounds_[2] >> cluster_bounds_[3];
            ROS_INFO("AP: \tcluster bounds: %f, %f, %f, %f", cluster_bounds_[0], cluster_bounds_[1], cluster_bounds_[2], cluster_bounds_[3]);
        }
        else if(lineNum == SAMPLESIZE) //third line specifies sample size for model in meters
        {
            lineParse >> sample_size_;
            ROS_INFO("AP: \tmodel sample size: %f", sample_size_);
        }
        else if(lineNum == CLUSTER_TOLERANCE)
        {
            lineParse >> cluster_tolerance_;
            ROS_INFO("AP: \tcluster tolerance: %f", cluster_tolerance_);
        }
        else if(lineNum == ON_TABLE)
        {
            lineParse >> on_table_;
            ROS_INFO("AP: \ton table: %d", on_table_);
        }
        else //all consecutive lines are model views
        {
            int id = lineNum - MODEL_VIEW_BEGIN;
            
            std::string pcd;
            float x,y,z,roll,pitch,yaw;
            lineParse >> pcd >> x >> y >> z >> roll >> pitch >> yaw;
            
            ROS_INFO("AP: \tloading view %s\n", pcd.c_str());
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
            pcl::io::loadPCDFile(db_path+obj_type_+"/"+pcd, *cloud);
            std::vector<int> idxUnusedVar;
            
            //be safe and remove NaN's and inf's (shouldn't be in this data in the first place)
            cloud->is_dense = false; //TODO: i don't trust the stored model data right now, setting is dense to false just makes sure that it checks whether the cloud has nan/inf points and removes them
            pcl::removeNaNFromPointCloud(*cloud, *cloud, idxUnusedVar);
            
            
            // //rotate andtranslate point cloud to have proper orientation (FIXME: data)
    //         pcl::transformPointCloud(*cloud, *cloud, pcl::getTransformation(0,2,0.9,90*M_PI/180.0,0,-90*M_PI/180.0));
            // //     pcl::transformPointCloud(*scene, *scene, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));
            // //     pcl::transformPointCloud(*model, *model, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));
            
            //downsample cloud
            pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
            pcl::VoxelGrid<PointT> vg;
            vg.setLeafSize(sample_size_, sample_size_, sample_size_);
            vg.setInputCloud(cloud);
            vg.filter(*cloud_filtered);
            
            //create new Model object and add to db
            Model newModel;
            newModel.cloud = cloud;
            newModel.cloud_sampled = cloud_filtered;
            newModel.id = id;
            newModel.pose.reset(new geometry_msgs::Pose);
            newModel.pose->position.x = x;
            newModel.pose->position.y = y;
            newModel.pose->position.z = z;
            newModel.pose->orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            models_.push_back(newModel);
        }
        
        ++lineNum;
    }
    ROS_INFO("AP: Done loading model database");

    return true;
}


