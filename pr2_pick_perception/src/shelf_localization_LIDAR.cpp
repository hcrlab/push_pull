#include "pr2_pick_perception/shelf_localization_lidar.h"
#include <boost/graph/graph_concepts.hpp>

#include <vtkTransform.h>

#include <laser_assembler/AssembleScans2.h>


///comparator for pair of double and int
bool pairComparator( const std::pair<double,int>& l, const std::pair<double,int>& r)
{ 
    return l.first < r.first; 
}


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "obj_detector");

    ros::NodeHandle nh;
    ObjDetector detector;    
    
    
    
    if(!detector.initialize())
    {
        ROS_FATAL("Object detector initialization failed. Shutting down node.");
        return 1;
    }

    ros::ServiceServer server(nh.advertiseService("localize_object", &ObjDetector::detectCallback, &detector));
  
  
    ros::spin();
    
    return 0;
}

ObjDetector::ObjDetector()
    : obj_type_("shelf"),
      min_cluster_size_(50),
      cluster_bounds_(4,0.0),
      sample_size_(0.05),
      cluster_tolerance_(0.05),
      score_thresh_(0.07),
      max_iter_(100),
      on_table_(false),
      debug_(false)
{
}


bool ObjDetector::initialize()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    
    
    
    //max iterations for ICP
    nh_local.getParam("iter", max_iter_);
    
    //min cluster points
    nh_local.param("minClusterSize", min_cluster_size_, 50);
    
    //score threshold for matching
    score_thresh_ = 0.08;//TODO: add 2 thresholds for rough and fine match?
    
    nh_local.param("radius_search", radius_search_, 0.03);
    nh_local.param("DistanceThreshold", dist_threshold_, 5.0);
    nh_local.param("PointColorThreshold", point_color_threshold_, 6.0);
    //nh_local.param("MinClusterSize", min_cluster_size_, 100.);
    nh_local.param("PlaneSize",PlaneSize_,2000);
    nh_local.param("PlanesegThres", PlanesegThres_,0.1);
    nh_local.param("highplane", highplane_,-1.0);
    nh_local.param("depthplane", depthplane_,-1.0);
    

    nh_local.param("manual_segmentation", manual_segmentation_,false);
    
    nh_local.param("RobotReference", robot_frame_id_,std::string("/base_footprint"));
    nh_local.param("WorldReference", world_frame_id_,std::string("/map"));
    nh_local.param("ModelReference", model_frame_id_,std::string("/model_frame"));
    
    
    
    nh_local.param("ColorSegmentation", color_segmentation_,false);
    
    nh_local.param("pca_alignment",pca_alignment_,false);
    
    nh_local.param("only_yaw",only_yaw_,false);
    nh_local.param("debug", debug_, false);
    
    nh_local.param("ICP2D",icp2D_, false);
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
    
     
    //advertise object detections
    std::string det_topic = nh.resolveName("/ap/detected_object");
    pub_ = nh.advertise<pr2_pick_perception::ObjectList>(det_topic, 1, true);     
    
    //ros::service::waitForService("/assemble_scans2");
    client_ = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");
    
   
    std::string camera_info = nh.resolveName("/cam_info");
    sensor_msgs::CameraInfoConstPtr cam_info;
    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info);//,nh,ros::Duration(15.));
    leftcamproj_.fromCameraInfo(cam_info);  
    camera_frame_id_ = cam_info->header.frame_id;
    im_size_.height = cam_info->height;
    im_size_.width = cam_info->width;    


    
 
    return true;
}



void ObjDetector::dataCbSync(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg,
        const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{

  // For sync error checking
  ++all_received_;  

  // call implementation
  dataCallback(image_msg, info_msg, pc_msg);
}


void ObjDetector::dataCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg,
        const sensor_msgs::PointCloud2ConstPtr& pc_msg){    
    
  
}


void ObjDetector::checkInputsSynchronized()
{
    int threshold = 3 * all_received_;
    if (image_received_ >= threshold || im_info_received_ >= threshold || stereo_pcl_received_ >= threshold)  {
        ROS_WARN("[road detector] Low number of synchronized image/image_info/stereo pcl tuples received.\n"
        "Images received:       %d (topic '%s')\n"
        "Camera info received:  %d (topic '%s')\n"       
        "Stereo point clouds received:      %d (topic '%s')\n"
        "Synchronized tuples: %d\n"
        "Possible issues:\n"
        "\t* stereo_image_proc is not running.\n"
        "\t  Does `rosnode info %s` show any connections?\n"
        "\t* The network is too slow. One or more images are dropped from each tuple.\n",
        image_received_, image_sub_.getTopic().c_str(),
                 im_info_received_, im_info_sub_.getTopic().c_str(),                 
                 stereo_pcl_received_, stereo_pcl_sub_.getTopic().c_str(),
                 all_received_, ros::this_node::getName().c_str());
    }
}
   

bool ObjDetector::detectCallback(pr2_pick_perception::LocalizeShelfRequest& request,
                                 pr2_pick_perception::LocalizeShelfResponse& response){
  
    
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = ros::Time(0);
    srv.request.end   = ros::Time::now();
    //subscribe to spinning lidar cloud service
    
    if (client_.call(srv))
    {
        ROS_DEBUG("AP: Got scene with %lu points.\n", srv.response.cloud.data.size());
    }
    else
    {
        ROS_ERROR("AP: Service call failed!\n");
        return false;
    }     
  
     //create point cloud from msg
    const sensor_msgs::PointCloud2 &cloud = srv.response.cloud;
    pcl::fromROSMsg(cloud, xtionPC_);
    
    cloud_frame_id_ = cloud.header.frame_id;
    
    
    //std::cout << "The name used to trigger is = " << request.object.obj_type << std::endl;
    if(request.object.obj_type != obj_type_)
    {
        return false;
    }
    else
    {
        ROS_INFO("AP: %s detection triggered (msg \"%s\")\n", obj_type_.c_str(), request.object.obj_type.c_str());
    }
    
    roi_.clear();
    manual_segmentation_ = false;
    
    if (request.object.region2d.bottom_right_x != -1 && request.object.region2d.bottom_right_y != -1 &&
        request.object.region2d.top_left_y != -1 && request.object.region2d.top_left_x != -1)   {
       manual_segmentation_ = true;
       roi_.push_back(cv::Point2f(request.object.region2d.top_left_x, request.object.region2d.top_left_y));
       roi_.push_back(cv::Point2f(request.object.region2d.bottom_right_x, request.object.region2d.bottom_right_y));
       
      
    }   
       
   
    
    std::string error_msg;    
    if (tf_.canTransform(robot_frame_id_, cloud_frame_id_, ros::Time(0), &error_msg)){
      tf_.lookupTransform(robot_frame_id_, cloud_frame_id_, ros::Time(0), cloud_to_robot_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            cloud_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_WARN("Transform error: %s", error_msg.c_str());
      cloud_to_robot_.setIdentity();
      robot_frame_id_ = cloud_frame_id_;
    }
    
     if (tf_.canTransform(camera_frame_id_, robot_frame_id_, ros::Time(0), &error_msg)){
      tf_.lookupTransform(camera_frame_id_, robot_frame_id_, ros::Time(0), robot_to_camera_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            camera_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_WARN("Transform error: %s", error_msg.c_str());
      robot_to_camera_.setIdentity();
      camera_frame_id_ = robot_frame_id_;
    }
    
    if (tf_.canTransform(world_frame_id_, robot_frame_id_,ros::Time(0), &error_msg)){
      tf_.lookupTransform(world_frame_id_, robot_frame_id_, ros::Time(0), robot_to_world_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            world_frame_id_.c_str(),robot_frame_id_.c_str());
      ROS_WARN("Transform error: %s", error_msg.c_str());
      robot_to_world_.setIdentity();
      world_frame_id_ = robot_frame_id_;
    }
    
   if (tf_.canTransform(model_frame_id_, cloud_frame_id_,ros::Time::now(), &error_msg)){
      tf_.lookupTransform(model_frame_id_, cloud_frame_id_,ros::Time::now(), cloud_to_model_);
    }
    else
    {
      ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to be available, " "will assume it as identity!",
            model_frame_id_.c_str(),cloud_frame_id_.c_str());
      ROS_WARN("Transform error: %s", error_msg.c_str());
      cloud_to_model_.setIdentity();
      model_frame_id_ = cloud_frame_id_;
    }
    
   
   
    
       /// Instead of transforming the detection, transform the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(xtionPC_,*scene,cloud_to_robot_); 
    
    
    //visualize scene
    pcl::visualization::PCLVisualizer::Ptr vis;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> scene_handler(scene, "x");//100, 100, 200);
    if (debug_) {
      vis.reset(new pcl::visualization::PCLVisualizer("Amchallenge -- Debug"));
      vis->addPointCloud(scene, scene_handler, "scene");
      vis->addCoordinateSystem();
      vis->setCameraPosition(-10,2,4,10,2,0,0,0,1);
      vis->spin();
    }
    
    
    ////////////////////////////////////
    // 1. extract clusters from scene //
    ////////////////////////////////////
    ROS_DEBUG("AP: clustering point cloud.\n");
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;    
    
    extractClusters(scene,&clusters);   
    
    //visualize clusters
    if (debug_) {
      vis->removePointCloud("scene");
      for(int i = 0; i < clusters.size(); ++i)
      {
         std::stringstream ss;
         ss << "cluster_raw_" << i;
         //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cluster_handler(clusters[i], 0, 0, 255.0);
         pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> cluster_handler(clusters[i]);
         vis->addPointCloud(clusters[i], cluster_handler, ss.str());
         vis->spinOnce();
      }
    }
    
    ////////////////////////////////////////////////////////////////
    // 2. perform simple filtering to discard impossible clusters //
    //    TODO: remove when switching to our cfvh?                //
    ////////////////////////////////////////////////////////////////
    
    ROS_INFO("Clusters detected = %d \n", (int)  clusters.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_candidates;
    for(int i = 0; i < clusters.size(); ++i)    {
      
    
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster = clusters[i];
        
        
        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cluster, minPt, maxPt);
        
        float height = maxPt.z-minPt.z;
        float width = std::max(maxPt.x-minPt.x, maxPt.y-minPt.y);
        //if(height <= OBJ_MAX_HEIGHT && height >= OBJ_MIN_HEIGHT &&  width <= OBJ_MAX_WIDTH && width >= OBJ_MIN_WIDTH) //(min height, min width, max height, max width) 
        ROS_INFO("width = %lf,  height =%lf",width,height);
    
    // cluster needs to satisfy min and max size constraints
        if(width >= cluster_bounds_[0] && width <= cluster_bounds_[1] && height >= cluster_bounds_[2] && height <= cluster_bounds_[3]
            && cluster->size() >= min_cluster_size_) 
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
    if (debug_) {
      vis->removePointCloud("scene");
      ROS_ERROR("%d cluster candidates", (int) cluster_candidates.size());
      for(int i = 0; i < cluster_candidates.size(); ++i)
      {   ROS_ERROR("cluster %d/%d", i, (int) cluster_candidates.size());
          std::stringstream ss;
          ss << "cluster_" << i;
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cluster_handler(cluster_candidates[i], 0, 100, 255.0*double(i)/cluster_candidates.size());    
          vis->addPointCloud(cluster_candidates[i], cluster_handler, ss.str());    
          vis->spinOnce();            
      }    
      vis->spin();
    }
    
    ROS_DEBUG("%d/%d clusters are candidates\n",(int) cluster_candidates.size(), (int) clusters.size());
    
    ////////////////////////////////////////////////
    // 3. run detection on each cluster candidate //
    ////////////////////////////////////////////////
    ROS_DEBUG("AP: starting detection ...\n");
    
    
    if (debug_) {
      pcl_ros::transformPointCloud(*scene,*scene,robot_to_world_); 
      vis->addPointCloud(scene, scene_handler, "scene");
    }
    
    std::vector<Model> detections;
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > detection_transforms;
    pcl::ScopeTime t_all;

    std::vector<double> scores;
    
    #pragma omp parallel for 
    for(int i = 0; i < cluster_candidates.size(); ++i)
    {
        ROS_DEBUG("AP: running detection on cluster %d\n", i);
        
        Eigen::Matrix4f detection_transform, detection_transform2;
        Eigen::Matrix3f R;
        Eigen::Vector3f tras;
         Eigen::Matrix4f TtoOrigin = Eigen::Matrix4f::Identity(); 
        int detection_id = -1;
        double score = 0.0;
        pcl::ScopeTime t;
        if (!icp2D_)
            detect(cluster_candidates[i], &detection_id, &score, &detection_transform, &detection_transform2);
        else
            detectICP2D(cluster_candidates[i], &detection_id, &score, &detection_transform, &detection_transform2);
        
        // transform the detections into the camera reference frame system

        
        //Copy the tf matrix to eigen format
        for (int j=0; j<3; j++)    
        {
            tras(j) = cloud_to_model_.inverse().getOrigin()[j];
            for (int k=0; k<3; k++)    
                R(j,k) = cloud_to_model_.inverse().getBasis()[j][k];
        }
        TtoOrigin.block<3,3>(0,0) = R; TtoOrigin.block<3,1>(0,3) = tras;
        
        std::cout <<"detected transform \n" << detection_transform << std::endl;
        std::cout << "Back to origin \n" << TtoOrigin << std::endl;
        
        
        detection_transform = TtoOrigin * detection_transform;
        detection_transform2 = TtoOrigin * detection_transform2;
        std::cout <<"after applying \n" << detection_transform << std::endl;
        
        
        
        ROS_DEBUG("Detection for cluster %d took %fs\n", (int) i, (float) t.getTimeSeconds());
        
        if(detection_id != -1)
        {
            Model m;
            getModel(detection_id, &m);

//             #pragma omp critical
            
                detections.push_back(m);
                detection_transforms.push_back(detection_transform);
                scores.push_back(score);
            
                //visualize 3D model of detection
            if (debug_) 
            {
                
          vtkSmartPointer<vtkTransform> vtkTrans(vtkTransform::New());
          const double vtkMat[16] = {(detection_transform)(0,0), (detection_transform)(0,1),(detection_transform)(0,2),(detection_transform)(0,3),
                                  (detection_transform)(1,0), (detection_transform)(1,1),(detection_transform)(1,2),(detection_transform)(1,3),
                                  (detection_transform)(2,0), (detection_transform)(2,1),(detection_transform)(2,2),(detection_transform)(2,3),
                                  (detection_transform)(3,0), (detection_transform)(3,1),(detection_transform)(3,2),(detection_transform)(3,3)};
          vtkTrans->SetMatrix(vtkMat);
          std::stringstream visMatchName;
          static int visMatchedModelCounter = 0;
          visMatchName << obj_type_ << visMatchedModelCounter++;
          ROS_ERROR("loading  %s for cluster %d", obj_type_.c_str(),i);
          std::string plyModel = ros::package::getPath("pr2_pick_perception")+"/models/"+obj_type_+"/"+obj_type_+".ply";
          vis->addModelFromPLYFile(plyModel, vtkTrans, visMatchName.str());
          
          //ROS_ERROR("added valve for cluster %d", i);
          
          pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloudMatched(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::transformPointCloud(*m.cloud, *modelCloudMatched, detection_transform2);
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_cloud_handler(modelCloudMatched, 0, 255, 0);
          vis->addPointCloud(modelCloudMatched, match_cloud_handler, visMatchName.str()+"_cloud");
          vis->addText3D(visMatchName.str(), pcl::PointXYZ(detection_transform(0,3),detection_transform(1,3),detection_transform(2,3)+2.5), 0.08, 1.0, 1.0, 1.0, visMatchName.str()+"_text");    
          vis->spinOnce();
        }
        
        }
        ROS_ERROR("cluster %d done", i);
    }
    ROS_DEBUG("Detection for all clusters took %fs\n", t_all.getTimeSeconds());
    
    ////////////////////////
    // 4. publish, yay :) //
    ////////////////////////    
    
    if (detections.size() == 0) {
      ROS_WARN("No shelf detected.");
      return false;
    }

    static int id_object=-1;
    pr2_pick_perception::ObjectList objects;
    for(int i = 0; i < detections.size(); ++i)
    {
        pr2_pick_perception::Object obj;
        obj.header.frame_id = world_frame_id_; 
        obj.header.stamp = pc_timestamp_; 
        std::stringstream ss;
        ss << "ap_" << obj_type_ <<  ++id_object;
        obj.id = ss.str();
        obj.object_ref = obj_type_; 
        obj.score = scores[i];
              
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
        
    }
    pub_.publish(objects);
    response.locations = objects;
    ROS_INFO("AP: published object list containing %d objects\n", (int)  objects.objects.size());

    if (debug_) {
      vis->spin();

    }

    return true;
 }

void ObjDetector::extractClusters(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& scene, std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > >* clusters){
  
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>) ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>) ;
     
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc_sampled (new pcl::PointCloud<pcl::PointXYZ>);
      
    
    std::cout << "Got colored point cloud  with " <<  scene->points.size() << "points" <<std::endl;
    
    //  Downsample 
    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    
    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (radius_search_);
    uniform_sampling.compute (sampled_indices);
    
    pcl::copyPointCloud (*scene, sampled_indices.points, *rgbpc_sampled);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << rgbpc_sampled->size () << std::endl;

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr rgbpc_sampled_cropped (new pcl::PointCloud<pcl::PointXYZ>);
     
    if(manual_segmentation_){
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
            if(roi.contains(im_point))// || rect2.contains(im_point) )
            {
                rgbpc_sampled_cropped->push_back(rgbpc_sampled->points[i]);     
               // std::cout << "point = [ " << im_point.x << "," << im_point.y << "]"<<std::endl;
            }
        }  
        
        //file.close();
    }
    else
    {
        //pcl::copyPointCloud (*rgbpc_sampled,*rgbpc_sampled_cropped);
        std::vector<int> sampled_indices2;
        for(size_t i=0;i<rgbpc_sampled->points.size();i++)
            sampled_indices2.push_back(i);
        pcl::copyPointCloud (*rgbpc_sampled,sampled_indices2 ,*rgbpc_sampled_cropped);
        std::cout << sampled_indices2.size() << std::endl;   
        
    }      
    
    
    std::cout << "Scene cropped points: " << rgbpc_sampled->size () << "; Sampled cropped Keypoints: " << rgbpc_sampled_cropped->size () << std::endl;
    
    /** get the wall and floor and eliminate from the scene ***/
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (PlanesegThres_);      
    seg.setMaxIterations (1000);

    /// Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i = 0;
    while (true) 
    {
    // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (rgbpc_sampled_cropped->makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the inliers
        extract.setInputCloud (rgbpc_sampled_cropped);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        fprintf(stderr, "plane size: %d, thresh: %d, small: %d\n", (int) cloud_p->points.size(), PlaneSize_, (int) (cloud_p->points.size() < PlaneSize_));
        if(cloud_p->points.size() < PlaneSize_) 
            break;
        std::cerr << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        rgbpc_sampled_cropped.swap (cloud_f);
        i++;
    }
      
     
       
      std::cout << "Point cloud cropped size = " << rgbpc_sampled_cropped->points.size() << std::endl;
     
       
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      
      /// Filter the points below certain height
      pass.setInputCloud (rgbpc_sampled_cropped);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (0.2, depthplane_);
        pass.filter (*scene_filtered);


        //height filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PassThrough<pcl::PointXYZ> pass2;
      
      /// Filter the points below certain height
      pass2.setInputCloud (scene_filtered);
      pass2.setFilterFieldName ("z");
      pass2.setFilterLimits (highplane_,3000);
      pass2.filter (*scene_filtered2);     
      
      
      std::cout << "Final point cloud without planes size = " << scene_filtered2->points.size() << std::endl;
      
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered_world (new pcl::PointCloud<pcl::PointXYZ>);
    /// Transfor the point cloud into world reference to give the object pose in world coordinates
    pcl_ros::transformPointCloud(*scene_filtered2,*scene_filtered_world,robot_to_world_); 
    
      ///extract clusters EUclidean
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(scene_filtered_world);    
    std::vector<pcl::PointIndices> clustersInd;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); 
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(scene_filtered_world);
    ec.extract(clustersInd);    
    
            
    for(int i = 0; i < clustersInd.size(); ++i) {      
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::PointXYZ point;
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
      
      
      
      
    if (debug_) {
      //pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = ec.reg.getColoredCloud ();
      pcl::visualization::CloudViewer viewer ("Cluster viewer");     
//       viewer.showCloud (rgbpc_sampled_cropped, "cloud cropped");
      viewer.showCloud (scene_filtered2, "scene_filtered");
      
      std::cout << "Starting visualization " << std::endl;

      while (!viewer.wasStopped ()){
          boost::this_thread::sleep (boost::posix_time::microseconds (1000));
      }
    }
  
    
}


void ObjDetector::detect2D(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster,  Eigen::Matrix4f* matchedTransform, 
                           Eigen::Matrix4f* matchedTransform2)
{

    std::vector<std::pair<double,int> > model_scores(models_.size()); //store model score with index for sorting
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_transforms(models_.size());

    //downsample cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(sample_size_, sample_size_, sample_size_);
    vg.setInputCloud(cluster);
    vg.filter(*cluster_sampled);
    ROS_INFO("cluster has %d points after sampling (%d before)\n",(int) cluster_sampled->size(), (int) cluster->size());

    int numThreads = models_.size(); //there's only up to 8 views per model at the moment so this runs 1 thread per view
    int schedule_chunk_size = 1;//round(models_.size()/8);
 
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr model = models_[0].cloud_sampled;

    ///////////////////////////////////////
    // 3. Shift clouds based on centroid //
    ///////////////////////////////////////
    Eigen::Vector4f centModel, centCluster;
    pcl::compute3DCentroid(*model, centModel);
    pcl::compute3DCentroid(*cluster_sampled, centCluster);
        /// Model transformed into the cluster reference system
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    
    Eigen::Matrix4f T_cluster = Eigen::Matrix4f::Identity(); 
        T_cluster.block<3,1>(0,3) = centCluster.head(3) - centModel.head(3);
    
    Eigen::Matrix4f T_model = Eigen::Matrix4f::Identity();

    
    pcl::transformPointCloud(*model, *model_transformed, T_cluster*T_model.inverse());   
    
    // 4. register clouds //
    ////////////////////////
    
    Eigen::Matrix4f transformation_matrix;
    pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ,float > estimateT;
    estimateT.estimateRigidTransformation (*cluster_sampled, *model_transformed,transformation_matrix);
    
    
    
    model_transforms[0] = transformation_matrix *  T_cluster*T_model.inverse();
     
    //calculate final transformation
    geometry_msgs::PosePtr mp = models_[0].pose;
    Eigen::Affine3f tr;
    tr = Eigen::Translation3f(mp->position.x, mp->position.y, mp->position.z) * Eigen::Quaternionf(mp->orientation.w, mp->orientation.x, mp->orientation.y, mp->orientation.z);
    Eigen::Matrix4f initPose(tr.data()); 
    
    std::cout << "Init model pose \n" << initPose << endl;
    
    *matchedTransform = /*refined_transform **/ model_transforms[0] * initPose;                         
    *matchedTransform2 = /*refined_transform **/ model_transforms[0];
    
    ROS_DEBUG_STREAM("AP: Transformation matrix: \n" << *matchedTransform);
    
}

void ObjDetector::detect(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster, int* matchedModelID, double *score, Eigen::Matrix4f* matchedTransform, Eigen::Matrix4f* matchedTransform2)
{
    //run thread for each model to speed up process - correct model should return after ~0.5s 
    //TODO: optimize, use our-cvfh to prune possible models, try to stop other threads once one of them found a match
    
    std::vector<std::pair<double,int> > model_scores(models_.size()); //store model score with index for sorting
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_transforms(models_.size());

    //downsample cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(sample_size_, sample_size_, sample_size_);
    vg.setInputCloud(cluster);
    vg.filter(*cluster_sampled);
    ROS_INFO("cluster has %d points after sampling (%d before)\n",(int) cluster_sampled->size(), (int) cluster->size());

    int numThreads = models_.size(); //there's only up to 8 views per model at the moment so this runs 1 thread per view
    int schedule_chunk_size = 1;//round(models_.size()/8);
    Eigen::Matrix3f  R_yaw, R_pitch,R_roll,R;

    
    
    
    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, schedule_chunk_size)
    for(int modelIdx = 0; modelIdx < models_.size(); ++modelIdx)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr model = models_[modelIdx].cloud_sampled;

        ///////////////////////////////////////
        // 3. Shift clouds based on centroid //
        ///////////////////////////////////////
        //TODO: maybe add eigen vectors (orientation)?
        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster_sampled, centCluster);
       // Eigen::Vector4f initialTransform = centCluster - centModel;               
       // Eigen::Quaternionf rot(1.0, 0.0, 0.0, 0.0);
        /// Model transformed into the cluster reference system
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        
        Eigen::Matrix4f T_cluster = Eigen::Matrix4f::Identity(); 
         T_cluster.block<3,1>(0,3) = centCluster.head(3) - centModel.head(3);
        
        Eigen::Matrix4f T_model = Eigen::Matrix4f::Identity();

        if (pca_alignment_)
        {
        
        //compute principal direction using eigenvectors computed by PCA class
            pcl::PCA<pcl::PointXYZ> pca;
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
        
        // 4. register clouds //
        ////////////////////////
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelRegistered(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::ScopeTime t("ICP...");
//         ROS_INFO("T%d:__6\n", omp_get_thread_num());
        pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
        //         pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration(new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>);
//         pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
        registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster_sampled);
        registration->setMaxCorrespondenceDistance(0.25);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(max_iter_);
        
        ROS_INFO("Input to ICP: inputSource: %dpts, inputTarget: %dpts\n",(int) model_transformed->size(), (int) cluster_sampled->size());

        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();
     

        Eigen::Matrix4f initT;

        
        ROS_INFO("score: %f\n", registration->getFitnessScore());
        model_scores[modelIdx] = std::make_pair(registration->getFitnessScore(), modelIdx);
     //   model_transforms[modelIdx] = transformation_matrix * initT;
        model_transforms[modelIdx] = transformation_matrix *  T_cluster*T_model.inverse();
        
    }

    //sort model scores (lower score = better)
    std::sort(model_scores.begin(), model_scores.end(), pairComparator);
        
    if(model_scores[0].first < score_thresh_)
    {
        *matchedModelID = model_scores[0].second;

        *score = model_scores[0].first;
        
        //calculate final transformation
        geometry_msgs::PosePtr mp = models_[*matchedModelID].pose;
        Eigen::Affine3f tr;
        tr = Eigen::Translation3f(mp->position.x, mp->position.y, mp->position.z) * Eigen::Quaternionf(mp->orientation.w, mp->orientation.x, mp->orientation.y, mp->orientation.z);
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
            
               


            matchedTransform->block<3,3>(0,0).setIdentity();
            matchedTransform->block<3,3>(0,0) = rot_desired;
            
        }
        *matchedTransform2 = /*refined_transform **/ model_transforms[*matchedModelID];
        
        ROS_DEBUG("AP: Best match: view %d with score of %f\n",(int) model_scores[0].second, model_scores[0].first);
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
        ROS_ERROR("AP: no model with id %d! (%d models in db)\n", id, (int) models_.size());
    }
}

void ObjDetector::detectICP2D(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster, int* matchedModelID, double *score,
                              Eigen::Matrix4f* matchedTransform, Eigen::Matrix4f* matchedTransform2)
{
    //run thread for each model to speed up process - correct model should return after ~0.5s 
    //TODO: optimize, use our-cvfh to prune possible models, try to stop other threads once one of them found a match
    
    std::vector<std::pair<double,int> > model_scores(models_.size()); //store model score with index for sorting
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_transforms(models_.size());

    //downsample cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(sample_size_, sample_size_, sample_size_);
    vg.setInputCloud(cluster);
    vg.filter(*cluster_sampled);
    ROS_INFO("cluster has %d points after sampling (%d before)\n",(int) cluster_sampled->size(), (int) cluster->size());

    int numThreads = models_.size(); //there's only up to 8 views per model at the moment so this runs 1 thread per view
    int schedule_chunk_size = 1;//round(models_.size()/8);
    Eigen::Matrix3f  R_yaw, R_pitch,R_roll,R;

    
    
    
    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, schedule_chunk_size)
    for(int modelIdx = 0; modelIdx < models_.size(); ++modelIdx)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr model = models_[modelIdx].cloud_sampled;

        ///////////////////////////////////////
        // 3. Shift clouds based on centroid //
        ///////////////////////////////////////
        //TODO: maybe add eigen vectors (orientation)?
        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster_sampled, centCluster);
       // Eigen::Vector4f initialTransform = centCluster - centModel;               
       // Eigen::Quaternionf rot(1.0, 0.0, 0.0, 0.0);
        /// Model transformed into the cluster reference system
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        
        Eigen::Matrix4f T_cluster = Eigen::Matrix4f::Identity(); 
         T_cluster.block<3,1>(0,3) = centCluster.head(3) - centModel.head(3);
        
        Eigen::Matrix4f T_model = Eigen::Matrix4f::Identity();

        if (pca_alignment_)
        {
        
        //compute principal direction using eigenvectors computed by PCA class
            pcl::PCA<pcl::PointXYZ> pca;
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
        
        // 4. register clouds //
        ////////////////////////
        pcl::PointCloud<pcl::PointXYZ>::Ptr modelRegistered(new pcl::PointCloud<pcl::PointXYZ>);
        
        pcl::ScopeTime t("ICP...");
        
         ROS_INFO("model transformed has %d points \n",(int) model_transformed->points.size());

        
        pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>::Ptr registration (new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>);

        boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> > warp_fcn 
        (new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);

         ROS_INFO("Input to ICP 1: \n");
        // Create a TransformationEstimationLM object, and set the warp to it
        boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
        te->setWarpFunction (warp_fcn);
        ROS_INFO("Input to ICP 2: \n");
         registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster_sampled);
        // Pass the TransformationEstimation objec to the ICP algorithm
        registration->setTransformationEstimation (te);
         ROS_INFO("Input to ICP opopopo \n");
       
        registration->setMaxCorrespondenceDistance(0.25);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(max_iter_);       

        
        
        ROS_INFO("Input to ICP: inputSource: %dpts, inputTarget: %dpts\n",(int) model_transformed->size(), (int) cluster_sampled->size());

        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();
     

        Eigen::Matrix4f initT;

        
        ROS_INFO("score: %f\n", registration->getFitnessScore());
        model_scores[modelIdx] = std::make_pair(registration->getFitnessScore(), modelIdx);
     //   model_transforms[modelIdx] = transformation_matrix * initT;
        model_transforms[modelIdx] = transformation_matrix *  T_cluster*T_model.inverse();
        
    }

    //sort model scores (lower score = better)
    std::sort(model_scores.begin(), model_scores.end(), pairComparator);
        
    if(model_scores[0].first < score_thresh_)
    {
        *matchedModelID = model_scores[0].second;
        *score = model_scores[0].first;
        
       
        
        //calculate final transformation
        geometry_msgs::PosePtr mp = models_[*matchedModelID].pose;
        Eigen::Affine3f tr;
        tr = Eigen::Translation3f(mp->position.x, mp->position.y, mp->position.z) * Eigen::Quaternionf(mp->orientation.w, mp->orientation.x, mp->orientation.y, mp->orientation.z);
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
            
               


            matchedTransform->block<3,3>(0,0).setIdentity();
            matchedTransform->block<3,3>(0,0) = rot_desired;
            
        }
        *matchedTransform2 = /*refined_transform **/ model_transforms[*matchedModelID];
        
        ROS_DEBUG("AP: Best match: view %d with score of %f\n",(int) model_scores[0].second, model_scores[0].first);
        ROS_DEBUG_STREAM("AP: Transformation matrix: \n" << *matchedTransform);
    }
    else
    {
        ROS_DEBUG("AP: No matches found for cluster (might be a good thing!)\n");
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

    std::string db_path = ros::package::getPath("pr2_pick_perception")+"/models/";
    
    
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
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile(db_path+obj_type_+"/"+pcd, *cloud) < 0)
            {
                ROS_ERROR("Failed loading model cloud \"%s\"!", (db_path+pcd).c_str());
                return false;
            }
            
            
            //be safe and remove NaN's and inf's (shouldn't be in this data in the first place)
            cloud->is_dense = false; //TODO: i don't trust the stored model data right now, setting is dense to false just makes sure that it checks whether the cloud has nan/inf points and removes them
            std::vector<int> idxUnusedVar;
            pcl::removeNaNFromPointCloud(*cloud, *cloud, idxUnusedVar);
            
  
            //downsample cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
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
