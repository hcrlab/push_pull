#include <obj_recognition/detectpc.h>
#include <math.h>
#define DEBUG //debugging output

#include <vtkTransform.h>


///comparator for pair of double and int
bool pairComparator( const std::pair<double,int>& l, const std::pair<double,int>& r)
{ 
    return l.first < r.first; 
}


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "obj_detector");   
  
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
:obj_type_(""), MinClusterSize_(50), cluster_bounds_(4,0.0), sample_size_(0.05), cluster_tolerance_(0.05), score_thresh_(0.07), max_iter_(100), on_table_(false), MinNeighborsInRadius_(50)
{

}


bool ObjDetector::initialize()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    
    //max iterations for ICP
    nh_local.getParam("iter", max_iter_);
    
    //min cluster points
    //nh_local.param("minClusterSize", min_cluster_size_, 50);
    
    //score threshold for matching
    score_thresh_ = 0.08;//TODO: add 2 thresholds for rough and fine match?
    
    nh_local.param("RadiusSearch", RadiusSearch_, 0.03);        
    //nh_local.param("RegionColorThreshold", RegionColorThreshold_, 3.0);
    nh_local.param("MinClusterSize", MinClusterSize_, 50.);
    nh_local.param("PlaneSize",PlaneSize_,2000);
    nh_local.param("PlanesegThres", PlanesegThres_,0.1);
    nh_local.param("highplane", highplane_,-1.0);
    nh_local.param("MinNeighborsInRadius", MinNeighborsInRadius_,10);
    

    nh_local.param("ManualSegmentation", ManualSegmentation_,false);
    
    nh_local.param("RobotReference", robot_frame_id_,std::string("/base_footprint"));
    nh_local.param("WorldReference", world_frame_id_,std::string("/map"));
    
    nh_local.param("ColorSegmentation", color_segmentation_,false);
    
    nh_local.param("pca_alignment",pca_alignment_,false);
    
    nh_local.param("only_yaw",only_yaw_,false);
    
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
    ros::service::waitForService("/assemble_scans2");
    client_ = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");
    
    //advertise object detections
    std::string det_topic = nh.resolveName("/ap/detected_object");
    pub_ = nh.advertise<ap_msgs::Object>(det_topic, 1, true); //TODO: latch or not latch?
    
    //subscribe to detection trigger topic
    std::string trigger_topic = nh.resolveName("/ap_ms/run_object_detection");
    trigger_sub_ = nh.subscribe<adaptive_perception_msgs::ObjectDetectionRequest>(trigger_topic, 1, &ObjDetector::detectCallback, this);
    ROS_INFO("subscribed to %s", trigger_sub_.getTopic().c_str());
 
    std::string camera_info = nh.resolveName("/cam_info");
    sensor_msgs::CameraInfoConstPtr cam_info;
    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info);//,nh,ros::Duration(15.));
    leftcamproj_.fromCameraInfo(cam_info);  
    camera_frame_id_ = cam_info->header.frame_id;
    im_size_.height = cam_info->height;
    im_size_.width = cam_info->width;        
 
    return true;
}


void ObjDetector::detectCallback(const adaptive_perception_msgs::ObjectDetectionRequestConstPtr &cmd){
  
    
    std::cout << "The name used to trigger is = " << cmd->obj_type.c_str() << std::endl;
    if(cmd->obj_type != obj_type_)
    {
        return;
    }
    else
    {
        ROS_INFO("AP: %s detection triggered (msg \"%s\")\n", obj_type_.c_str(), cmd->obj_type.c_str());
    }
    roi_.clear();
    ManualSegmentation_ = false;
    if (cmd->region2d.bottom_right_x != -1 && cmd->region2d.bottom_right_y != -1 &&
        cmd->region2d.top_left_y != -1 && cmd->region2d.top_left_x != -1)   
    {
       ManualSegmentation_ = true;
       roi_.push_back(cv::Point2f(cmd->region2d.top_left_x, cmd->region2d.top_left_y));
       roi_.push_back(cv::Point2f(cmd->region2d.bottom_right_x, cmd->region2d.bottom_right_y));        
    }
    
  
     //////////////////
    // 0. get scene //
    //////////////////
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = ros::Time(0);
    srv.request.end   = ros::Time::now();
    if (client_.call(srv))
    {
        ROS_DEBUG("AP: Got scene with %lu points.\n", srv.response.cloud.data.size());
    }
    else
    {
        ROS_ERROR("AP: Service call failed!\n");
        return;
    }     
  
  
  
   //create point cloud from msg
    const sensor_msgs::PointCloud2 &cloud = srv.response.cloud;
    pcl::PointCloud<PointT>::Ptr scene_world(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud, *scene_world);
    
   
    cloud_frame_id_ = cloud.header.frame_id;
    std::string error_msg;    
    if (tf_.canTransform(robot_frame_id_, cloud_frame_id_, cloud.header.stamp, &error_msg)){
      tf_.lookupTransform(robot_frame_id_, cloud_frame_id_, cloud.header.stamp, cloud_to_robot_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            cloud_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      cloud_to_robot_.setIdentity();
    }
    
     if (tf_.canTransform(camera_frame_id_, robot_frame_id_, cloud.header.stamp, &error_msg)){
      tf_.lookupTransform(camera_frame_id_, robot_frame_id_, cloud.header.stamp, robot_to_camera_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            camera_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      robot_to_camera_.setIdentity();
    }
    
    if (tf_.canTransform(world_frame_id_, robot_frame_id_, cloud.header.stamp, &error_msg)){
      tf_.lookupTransform(world_frame_id_, robot_frame_id_, cloud.header.stamp, robot_to_world_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            world_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      robot_to_world_.setIdentity();
    }
    
    /// Instead of transforming the detection, transform the point cloud
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl_ros::transformPointCloud(*scene_world,*scene,cloud_to_robot_); 
    
    
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
    
    extractClusters(scene,&clusters);   
    //pcl::io::savePCDFileASCII ("/home/lpuig/trooper_clean_3d_obj/scene_pcd.pcd", *scene);
    

    //visualize clusters
#ifdef DEBUG
     vis->removePointCloud("scene");
     for(int i = 0; i < clusters.size(); ++i)
     {
        std::stringstream ss;
        ss << "cluster_raw_" << i;
        //pcl::visualization::PointCloudColorHandlerCustom<PointTc> cluster_handler(clusters[i], 0, 0, 255.0);
        pcl::visualization::PointCloudColorHandlerRandom<PointT> cluster_handler(clusters[i]);
        vis->addPointCloud(clusters[i], cluster_handler, ss.str());
        vis->spinOnce();
     }
    
#endif
    
    ////////////////////////////////////////////////////////////////
    // 2. perform simple filtering to discard impossible clusters //
    //    TODO: remove when switching to our cfvh?                //
    ////////////////////////////////////////////////////////////////
    
    ROS_INFO("Clusters detected = %d \n",  clusters.size());
    std::vector<pcl::PointCloud<PointT>::Ptr> cluster_candidates;
    for(int i = 0; i < clusters.size(); ++i)    {
      
        pcl::PointCloud<PointT>::Ptr cluster = clusters[i];        
        
        PointT minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);
        
        float height = maxPt.z-minPt.z;
        float width = std::max(maxPt.x-minPt.x, maxPt.y-minPt.y);
        //if(height <= OBJ_MAX_HEIGHT && height >= OBJ_MIN_HEIGHT &&  width <= OBJ_MAX_WIDTH && width >= OBJ_MIN_WIDTH) //(min height, min width, max height, max width) 
        ROS_INFO("width = %lf,  height =%lf",width,height);
    
    // cluster needs to satisfy min and max size constraints
        if(width >= cluster_bounds_[0] && width <= cluster_bounds_[1] && height >= cluster_bounds_[2] && height <= cluster_bounds_[3]
      && cluster->size() >= MinClusterSize_) 
        {
            cluster_candidates.push_back(cluster);
            ROS_INFO("cluster candidate added (h:%f,w:%f)\n", height, width);
        }
        else
        {
       //      ROS_ERROR("%f<=%f<=%f or %f<=%f<=%f invalid\n", OBJ_MIN_HEIGHT, height, OBJ_MAX_HEIGHT, OBJ_MIN_WIDTH, width, OBJ_MAX_WIDTH);
        }
    }
    
    //visualize cluster candidates
#ifdef DEBUG
    vis->removePointCloud("scene");
    ROS_ERROR("%d cluster candidates", cluster_candidates.size());
    for(int i = 0; i < cluster_candidates.size(); ++i)
    {
        ROS_ERROR("cluster %d/%d", i, cluster_candidates.size());
        std::stringstream ss;
        ss << "cluster_" << i;
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cluster_handler(cluster_candidates[i], 0, 100, 255.0*double(i)/cluster_candidates.size());
        ROS_ERROR("1");
        vis->addPointCloud(cluster_candidates[i], cluster_handler, ss.str());
        ROS_ERROR("2");
        vis->spinOnce();
    }
    ROS_ERROR("3");
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
    std::vector<pcl::PointCloud<PointT>::Ptr > detections_aligned;
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
                pcl::PointCloud<PointT>::Ptr model_aligned(new pcl::PointCloud<PointT>);
                pcl::transformPointCloud(*m.cloud, *model_aligned, detection_transform2);

                detections_aligned.push_back(model_aligned);
            
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
                visMatchName << obj_type_ << visMatchedModelCounter++;
                //ROS_ERROR("loading valve for cluster %d", i);
                std::string plyModel = ros::package::getPath("obj_recognition")+"/models/"+obj_type_+"/"+obj_type_+".ply";
                vis->addModelFromPLYFile(plyModel, vtkTrans, visMatchName.str());
               // ROS_ERROR("added valve for cluster %d", i);
                
                pcl::PointCloud<PointT>::Ptr modelCloudMatched(new pcl::PointCloud<PointT>);
                pcl::transformPointCloud(*m.cloud, *modelCloudMatched, detection_transform2);
                pcl::visualization::PointCloudColorHandlerCustom<PointT> match_cloud_handler(modelCloudMatched, 255, 0, 0);
                vis->addPointCloud(modelCloudMatched, match_cloud_handler, visMatchName.str()+"_cloud");
                vis->addText3D(visMatchName.str(), PointT(detection_transform(0,3),detection_transform(1,3),detection_transform(2,3)+2.5), 0.08, 1.0, 1.0, 1.0, visMatchName.str()+"_text");    
                vis->spinOnce();
#endif
            }
        }
        ROS_ERROR("cluster %d done", i);
    }
    ROS_DEBUG("Detection for all clusters took %fs\n", t_all.getTimeSeconds());
    
    
    std::vector<pcl::PointCloud<PointT>::ConstPtr> aligned_models;   
    aligned_models.resize (detections_aligned.size ());
    for (int i=0; i<detections_aligned.size (); i++)
    {
        aligned_models[i] = detections_aligned[i];
    }
        //configure hypothesis verificator
    pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> papazov;
    papazov.setResolution (0.005f);
    papazov.setInlierThreshold (0.005f);
    papazov.setSupportThreshold (0.08f);
    papazov.setPenaltyThreshold (0.05f);
    papazov.setConflictThreshold (0.02f);
    papazov.setOcclusionThreshold (0.01f);
  //  papazov.addModels (aligned_models, true);
  //  papazov.verify ();
   // std::vector<bool> mask_hv1;
   // papazov.getMask (mask_hv1);    
    //  papazon.addModels(detections_aligned, true);

    pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> greedy;
    greedy.setResolution (0.01f);
    greedy.setInlierThreshold (0.005f);
    greedy.setOcclusionThreshold (0.01f);


    greedy.setSceneCloud (scene);
    greedy.addModels (aligned_models, true);
    greedy.verify ();
    std::vector<bool> mask_hv;
    greedy.getMask (mask_hv);    
  
    
    
  
    ////////////////////////
    // 4. publish, yay :) //
    ////////////////////////
    
    
    static int id_object=-1;
    ap_msgs::ObjectList objects;
    for(int i = 0; i < mask_hv.size(); ++i)
    {
        if (!mask_hv[i])
        {
        
            ap_msgs::Object obj;
            obj.header.frame_id = world_frame_id_; 
            obj.header.stamp = srv.request.end;        
            std::stringstream ss;
            ss << "ap_" << obj_type_ <<  ++id_object;
            obj.id = ss.str();
            obj.object_ref = obj_type_;        
          //  obj.wm_operation.action = trooper_msgs::WorldModelOperation::ADD;
          //  obj.wm_authority.value = trooper_msgs::WorldModelAuthor::HUMAN;
                
            obj.pose.position.x = detection_transforms[i](0,3);
            obj.pose.position.y = detection_transforms[i](1,3); 
            obj.pose.position.z = detection_transforms[i](2,3); 
            
            Eigen::AngleAxisf angax;
            angax = detection_transforms[i].topLeftCorner<3,3>();
            Eigen::Quaternionf quat(detection_transforms[i].topLeftCorner<3,3>());
                
            obj.pose.orientation.x = quat.x();
            obj.pose.orientation.y = quat.y();
            obj.pose.orientation.z = quat.z();
            obj.pose.orientation.w = quat.w();
            
            objects.objects.push_back(obj);
            pub_.publish(obj);
        }
    }   
    ROS_INFO("AP: published object list containing %d objects\n", objects.objects.size());
    ROS_INFO("AP: Total objects detected %d objects\n", mask_hv.size());
    
    
#ifdef DEBUG
    vis->spin();
//     ROS_ERROR("VISUALIZER IS DEAD!!!!\n");
//     vis->close();
// //     vis.reset();
//     ROS_ERROR("CLOSED?\n");
#endif    

  
 }

void ObjDetector::extractClusters(const pcl::PointCloud< PointT >::ConstPtr& scene, std::vector< boost::shared_ptr< pcl::PointCloud< PointT > > >* clusters){
  
  
    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>) ;
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>) ;
     
    pcl::PointCloud<PointT>::Ptr rgbpc (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr rgbpc_sampled (new pcl::PointCloud<PointT>);
      
    
    std::cout << "Got point cloud  with " <<  scene->points.size() << "points" <<std::endl;
    
    //  Downsample 
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<PointT> uniform_sampling;
    
    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (RadiusSearch_);
    uniform_sampling.compute (sampled_indices);
    
    pcl::copyPointCloud (*scene, sampled_indices.points, *rgbpc_sampled);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << rgbpc_sampled->size () << std::endl;

    
    pcl::PointCloud<PointT>::Ptr rgbpc_sampled_cropped (new pcl::PointCloud<PointT>);
     
    if(ManualSegmentation_)
    {
       cv::Rect roi(roi_[0].x*im_size_.width,roi_[0].y*im_size_.height, 
                     (roi_[1].x - roi_[0].x)*im_size_.width , (roi_[1].y - roi_[0].y)*im_size_.height); 
        
        for(size_t i=0;i<rgbpc_sampled->points.size();i++)
        {
            //tranform this point to the camera reference system
            tf::Point tfX(rgbpc_sampled->points[i].x, rgbpc_sampled->points[i].y, rgbpc_sampled->points[i].z);  
           
            tf::Point tfXcam =  robot_to_camera_ * tfX;            

            //find rgb value for this point in the image                  
            cv::Point3d xyz( tfXcam.m_floats[0], tfXcam.m_floats[1], tfXcam.m_floats[2]); //a Point3d of the point in the pointcloud
            cv::Point2d im_point;  //will hold the 2d point in the image
            im_point = leftcamproj_.project3dToPixel(xyz);            
            
           // file <<  im_point.x << "," << im_point.y << ", " << norm << std::endl;
            if(roi.contains(im_point))
            {
                rgbpc_sampled_cropped->push_back(rgbpc_sampled->points[i]);     
               // std::cout << "point = [ " << im_point.x << "," << im_point.y << "]"<<std::endl;
            }
        }           
    }
    else
    {
        rgbpc_sampled_cropped = rgbpc_sampled;
    }          
    
    
    //pcl::io::savePCDFileASCII ("/home/lpuig/trooper_clean_3d_obj/cropped_scene_pcd.pcd", *rgbpc_sampled_cropped);
    
    
    /*****    Filter the point cloud    *****/
    // point cloud instance for the result
    
    pcl::PointCloud<PointT>::Ptr pc_cleaned (new pcl::PointCloud<PointT>);

    // create the radius outlier removal filter
    pcl::RadiusOutlierRemoval<PointT> radius_outlier_removal;

    // set input cloud
    radius_outlier_removal.setInputCloud (rgbpc_sampled_cropped);

    // set radius for neighbor search
    radius_outlier_removal.setRadiusSearch (0.05);

    // set threshold for minimum required neighbors neighbors
    radius_outlier_removal.setMinNeighborsInRadius (MinNeighborsInRadius_);     
    
    // do filtering
    radius_outlier_removal.filter (*pc_cleaned);
    
    
    /** get the wall and floor and eliminate from the scene ***/    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (PlanesegThres_);      
    seg.setMaxIterations (1000);

    /// Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    
    while (true)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (pc_cleaned->makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (pc_cleaned);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        fprintf(stderr, "plane size: %d, thresh: %d, small: %d\n", cloud_p->points.size(), PlaneSize_, (cloud_p->points.size() < PlaneSize_));
        if(cloud_p->points.size() < PlaneSize_) 
            break;
       // std::cerr << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        pc_cleaned.swap (cloud_f);
    
    }        
       
   // std::cout << "Point cloud cropped size = " << pc_cleaned->points.size() << std::endl;          
      
    pcl::PointCloud<PointT>::Ptr scene_filtered(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    
    /// Filter the points below certain height
    pass.setInputCloud (pc_cleaned);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (highplane_, 100.0);
    pass.filter (*scene_filtered);          
      
    //std::cout << "Final point cloud without planes size = " << scene_filtered->points.size() << std::endl;
      
    pcl::PointCloud<PointT>::Ptr scene_filtered_world (new pcl::PointCloud<PointT>);
    /// Transfor the point cloud into world reference to give the object pose in world coordinates
    pcl_ros::transformPointCloud(*scene_filtered,*scene_filtered_world,robot_to_world_); 
    
    ///extract clusters EUclidean
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(scene_filtered_world);    
    std::vector<pcl::PointIndices> clustersInd;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_); //25cm
    ec.setMinClusterSize(MinClusterSize_); //need to go low for standpipe
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(scene_filtered_world);
    ec.extract(clustersInd);        
            
    for(int i = 0; i < clustersInd.size(); ++i) {      
        pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>); 
        PointT point;
        for(int j=0; j < clustersInd[i].indices.size();j++){
            int index = clustersInd[i].indices[j];
            point.x = scene_filtered_world->points[index].x;
            point.y = scene_filtered_world->points[index].y;
            point.z = scene_filtered_world->points[index].z;      
            cluster->points.push_back(point);     
        }
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        clusters->push_back(cluster);
    }      
      
#ifdef DEBUG
      //pcl::PointCloud <PointTc>::Ptr colored_cloud = ec.reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");     
    // viewer.showCloud (rgbpc_sampled_cropped, "cloud cropped");
    viewer.showCloud (scene_filtered, "scene_filtered");
    
    std::cout << "Starting visualization " << std::endl;

    while (!viewer.wasStopped ()){
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
#endif  
    
}



void ObjDetector::detect(const pcl::PointCloud< PointT >::ConstPtr& cluster, int* matchedModelID, Eigen::Matrix4f* matchedTransform, 
                         Eigen::Matrix4f* matchedTransform2, const pcl::visualization::PCLVisualizer::Ptr &visu)
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
    //ROS_INFO("cluster has %d points after sampling (%d before)\n", cluster_sampled->size(), cluster->size());

    int numThreads = models_.size(); //there's only up to 8 views per model at the moment so this runs 1 thread per view
    int schedule_chunk_size = 1;//round(models_.size()/8);
    Eigen::Matrix3f  R_yaw, R_pitch,R_roll,R;

    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, schedule_chunk_size)
    for(int modelIdx = 0; modelIdx < models_.size(); ++modelIdx)
    {
        pcl::PointCloud<PointT>::Ptr model = models_[modelIdx].cloud_sampled;
//         ROS_INFO("T%d:__3.5\n", omp_get_thread_num());
        
        ///////////////////////////////////////
        // 3. Shift clouds based on centroid //
        ///////////////////////////////////////
        
        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster_sampled, centCluster);
       
        // Model transformed into the cluster reference system
        pcl::PointCloud<PointT>::Ptr model_transformed(new pcl::PointCloud<PointT>);
        
        Eigen::Matrix4f T_cluster = Eigen::Matrix4f::Identity(); 
        T_cluster.block<3,1>(0,3) = centCluster.head(3) - centModel.head(3);
        
        Eigen::Matrix4f T_model = Eigen::Matrix4f::Identity();

        if (pca_alignment_)
        {        
            //compute principal direction using eigenvectors computed by PCA class
            pcl::PCA<PointT> pca;
            pca.setInputCloud(model);
            Eigen::Matrix3f R_model = pca.getEigenVectors();
            Eigen::Vector3f t_model = pca.getMean().head(3);

            
            T_model.block<3,3>(0,0) = R_model;
            T_model.block<3,1>(0,3) = t_model;
            
            if (R_model.determinant() < 0.0)
            {            
                T_model.setIdentity();
                T_model.row(1) << R_model.row(0),0.0; 
                T_model.row(2) << R_model.row(1),0.0; 
                T_model.row(0) << (R_model.row(0).cross(R_model.row(1))).normalized(),0;             
                T_model.block<3,1>(0,3) = t_model;        
            }
            pca.setInputCloud(cluster_sampled);
            Eigen::Matrix3f R_cluster = pca.getEigenVectors();
            Eigen::Vector3f t_cluster = pca.getMean().head(3);       
            
    //         Eigen::Vector3f t_cluster = centCluster.head(3);
    //         Eigen::Matrix3f covariance_cluster;
    //         pcl::computeCovarianceMatrixNormalized(*cluster_sampled,centCluster, covariance_cluster);
    //         Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver_cluster(covariance_cluster, Eigen::ComputeEigenvectors);
    //         Eigen::Matrix3f R_cluster = eigen_solver_cluster.eigenvectors();
    //         R_cluster.col(2) = R_cluster.col(0).cross(R_cluster.col(1));        
            
            T_cluster.block<3,3>(0,0) = R_cluster;
            T_cluster.block<3,1>(0,3) = t_cluster;
            
            if (R_cluster.determinant() < 0.0)
            {
                    T_cluster.setIdentity();
                    T_cluster.col(1) << R_cluster.col(0),0; 
                    T_cluster.col(2) << R_cluster.col(1),0; 
                    T_cluster.col(0) << (R_cluster.col(0).cross(R_cluster.col(1))).normalized(),0;           
                    T_cluster.block<3,1>(0,3) = t_cluster;
            }     

        }
    
        pcl::transformPointCloud(*model, *model_transformed, T_cluster*T_model.inverse());   
        //pcl::transformPointCloud(*model, *model_transformed, Eigen::Vector3f(initialTransform[0],initialTransform[1],initialTransform[2]), rot);
        
        ////////////////////////
        // 4. register clouds //
        ////////////////////////
        pcl::PointCloud<PointT>::Ptr modelRegistered(new pcl::PointCloud<PointT>);
        
        pcl::ScopeTime t("ICP...");
        // ROS_INFO("T%d:__6\n", omp_get_thread_num());
        pcl::Registration<PointT, PointT>::Ptr registration (new pcl::IterativeClosestPoint<PointT, PointT>);
        // pcl::Registration<PointT, PointT>::Ptr registration(new pcl::IterativeClosestPointNonLinear<PointT, PointT>);
        // pcl::Registration<PointT, PointT>::Ptr registration(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>);
        registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster_sampled);
        registration->setMaxCorrespondenceDistance(0.25);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(max_iter_);
        
        ROS_INFO("Input to ICP: inputSource: %dpts, inputTarget: %dpts\n", model_transformed->size(), cluster_sampled->size());

        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();     
        
        ROS_INFO("score: %f\n", registration->getFitnessScore());
        model_scores[modelIdx] = std::make_pair<double,int>(registration->getFitnessScore(), modelIdx);
     
        model_transforms[modelIdx] = transformation_matrix *  T_cluster*T_model.inverse();
        
    }

    //sort model scores (lower score = better)
    std::sort(model_scores.begin(), model_scores.end(), pairComparator);
        
    if(model_scores[0].first < score_thresh_)
    {
        *matchedModelID = model_scores[0].second;        
        
        //calculate final transformation
        geometry_msgs::PosePtr mp = models_[*matchedModelID].pose;
        Eigen::Affine3f tr;
        tr = Eigen::Translation3f(mp->position.x, mp->position.y, mp->position.z) 
             * Eigen::Quaternionf(mp->orientation.w, mp->orientation.x, mp->orientation.y, mp->orientation.z);
        Eigen::Matrix4f initPose(tr.data()); //FIXME: does this actually work?
        
        std::cout << "Init model pose \n" << initPose << endl;
        
        *matchedTransform = /*refined_transform **/ model_transforms[*matchedModelID] * initPose;                         
        Eigen::Matrix3f  R_pitch_init,R_roll_init;
        if(only_yaw_)
        {
            Eigen::Vector3f ypr,ypr_init;
          //  Eigen::Matrix3f  R_yaw, R_pitch,R_roll,R;
            ypr = matchedTransform->block<3,3>(0,0).eulerAngles(2,1,0);
            ypr_init = initPose.block<3,3>(0,0).eulerAngles(2,1,0);
            
            Eigen::Matrix3f rot_desired;
            rot_desired = Eigen::AngleAxisf(ypr(0), Eigen::Vector3f::UnitZ()) 
                        * Eigen::AngleAxisf(ypr_init(1), Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(ypr_init(2), Eigen::Vector3f::UnitX());
            
   ///         R_yaw << cos(ypr(0)), -sin(ypr(0)),0, sin(ypr(0)),cos(ypr(0)),0,0,0,1;
/*            R_pitch << cos(ypr(1)), 0,  sin(ypr(1)),0, 1,0, -sin(ypr(1)),0,cos(ypr(1));
            R_roll << 1, 0 ,0 ,0, cos(ypr(2)), -sin(ypr(2)),0, sin(ypr(2)),cos(ypr(2));    */                


            matchedTransform->block<3,3>(0,0).setIdentity();
            matchedTransform->block<3,3>(0,0) = rot_desired;
            
        }
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
    
    ROS_INFO("reading file = %s",db_file.c_str());
    if(!file.is_open())
    {
        ROS_ERROR("File \"%s\" not found!\n", db_file.c_str());
        return false;
    }

    std::string db_path = ros::package::getPath("obj_recognition")+"/models/";
    
    
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
            if(pcl::io::loadPCDFile(db_path+obj_type_+"/"+pcd, *cloud) < 0)
            {
                ROS_ERROR("Failed loading model cloud \"%s\"!", (db_path+pcd).c_str());
                return false;
            }
            
            
            //be safe and remove NaN's and inf's (shouldn't be in this data in the first place)
            cloud->is_dense = false; //TODO: i don't trust the stored model data right now, setting is dense to false just makes sure that it checks whether the cloud has nan/inf points and removes them
            std::vector<int> idxUnusedVar;
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


void ObjDetector::getTransformationFromCorrelation ( const Eigen::MatrixXd &cloud_src_demean, const Eigen::Vector4d & centroid_src,
        const Eigen::MatrixXd &cloud_tgt_demean, const Eigen::Vector4d & centroid_tgt, Eigen::Matrix4d &transformation_matrix){
            
     // Assemble the correlation matrix H = source * target'
  //Eigen::Matrix3d H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner (3, 3);   
  
    
  Eigen::MatrixXd H = cloud_src_demean.transpose() * cloud_tgt_demean;
  
  
  std::cout << "H = \n"  << H << std::endl;
  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd u = svd.matrixU ();
  Eigen::MatrixXd v = svd.matrixV ();

  // Compute R = V * U'
//   if (u.determinant () * v.determinant () < 0)
//   {
//     for (int x = 0; x < v.rows(); ++x)
//       v (x, 2) *= -1;
//   }

  Eigen::Matrix3d R = v * u.transpose ();

  // Return the correct transformation
  transformation_matrix.topLeftCorner (3, 3) = R;
  const Eigen::Vector3d Rc (R * centroid_src.head (3));
  transformation_matrix.block (0, 3, 3, 1) = centroid_tgt.head (3) - Rc;
    
    
}
