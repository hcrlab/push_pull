#include <trooper_obj_recognition/selectobject.h>


Obj_selector::Obj_selector()
:flag_service_(false), image_ready_(false)
{}


void Obj_selector::dataCbSync(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg,
	    const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{

  // For sync error checking
  ++all_received_;  

  // call implementation
  dataCallback(image_msg, info_msg, pc_msg);
}


void Obj_selector::dataCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::CameraInfoConstPtr& info_msg,
	    const sensor_msgs::PointCloud2ConstPtr& pc_msg){
  
  if (flag_service_)  {
    stereo_ref_system_ = pc_msg->header.frame_id;
    pcl::fromROSMsg(*pc_msg, stereo_pcl_);  
  
    leftcamproj_.fromCameraInfo(info_msg);
    l_cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    camera_frame_id_ = info_msg->header.frame_id; 
    l_cv_ptr_->image.copyTo(frame_);  
    
    image_ready_ = true;
    flag_service_ = false;
  }
    
    
}


void Obj_selector::checkInputsSynchronized()
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

      
      
bool Obj_selector::selectObjectCb( trooper_adaptive_perception_msgs::SelectObject::Request &request, 
		  trooper_adaptive_perception_msgs::SelectObject::Response &response){
		      
    flag_service_ = true;
  
      int queue_size = 2; 
    // subscribe to left image
    image_transport::ImageTransport it(nh_);
    std::string topicImleft = nh_.resolveName("/image");    
    image_sub_.subscribe(it, topicImleft, queue_size); 
    std::string topicInfocam = nh_.resolveName("/cam_info");
    im_info_sub_.subscribe(nh_, topicInfocam, queue_size);
    std::string stereo_topic = nh_.resolveName("/stereo_cloud");
    stereo_pcl_sub_.subscribe(nh_, stereo_topic, queue_size);    
	
    image_sub_.registerCallback(boost::bind(increment, &image_received_));
    im_info_sub_.registerCallback(boost::bind(increment, &im_info_received_));
    stereo_pcl_sub_.registerCallback(boost::bind(increment, &stereo_pcl_received_));
    
    //exact sync is required as disparity image is calculated from left image and has to have the same timestamp
    check_synced_timer_ = nh_.createTimer(ros::Duration(15.0), boost::bind(&Obj_selector::checkInputsSynchronized,this));
    
    
    exact_sync_.reset(new ExactSync(ExactPolicy(queue_size), image_sub_, im_info_sub_, stereo_pcl_sub_));
    exact_sync_->registerCallback(boost::bind(&Obj_selector::dataCbSync,this,  _1, _2, _3));
  
    
    while(!image_ready_)
    {
      ros::Duration waitTimer(0,1000);
      waitTimer.sleep();
    }
  
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
        return false;
    }     
  
  
  
   //create point cloud from msg
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud);
    pcl::PointCloud<PointT>::Ptr scene_world(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(cloud, *scene_world);
    
   
    cloud_frame_id_ = cloud.header.frame_id;
    
    std::string error_msg;//tf_stereo_
    tf_.waitForTransform(world_frame_id_, stereo_ref_system_,ros::Time(),ros::Duration(10.0), ros::Duration(0.0), &error_msg);
    tf_.lookupTransform(world_frame_id_,stereo_ref_system_,ros::Time(), stereo2world_);    
    
    
    ///Transform the point cloud to world frame    
    pcl::PointCloud<PointT> outCloud;
    pcl_ros::transformPointCloud(stereo_pcl_,outCloud,stereo2world_);
    cv::Size im_size = frame_.size(); 
        
    cv::Mat mask = cv::Mat::zeros(im_size.height+2,im_size.width+2,CV_8UC1);
    cv::Vec3b blackpix(-1,-1,-1);
    /// remove the ground from the image using the point cloud
    //ROS_ERROR("Got n points from stereo %d", outCloud.points.size());
    
    for(int i=0;i<outCloud.points.size();i++){	  
      boost::mutex::scoped_lock  lock(stereo_mtx_);
      cv::Point3d wxyz(outCloud.points[i].x, outCloud.points[i].y, outCloud.points[i].z); //a Point3d of the point in the pointcloud
      cv::Point3d cxyz(stereo_pcl_.points[i].x, stereo_pcl_.points[i].y, stereo_pcl_.points[i].z); //a Point3d of the point in the pointcloud
      //std::cout << "[" << wxyz.x << "," << wxyz.y << "," << wxyz.z << "]" << std::endl;
      cv::Point2d im_point;  //will hold the 2d point in the image
      im_point = leftcamproj_.project3dToPixel(cxyz);
      if (im_point.x > 0 && im_point.x < im_size.width && im_point.y > 0 && im_point.y < im_size.height && outCloud.points[i].z < 0.2){
      //std::cout << "[ " << (int)im_point.y <<","<<(int)im_point.x << "] = [" << cxyz.x << "," << cxyz.y << "," << cxyz.z << "]" << std::endl;
	//frame_.at<cv::Vec3b>(floor(im_point.y+0.5),floor(im_point.x+0.5)) = blackpix;	
	mask.at<uchar>(floor(im_point.y+0.5)+1,floor(im_point.x+0.5)+1) = 255;
      }
    }
    
      
      
      /// change colorspace      
      cv::Mat tmp,gray;      
      cv::cvtColor(frame_,tmp,CV_BGR2HSV);
      cv::cvtColor(frame_,gray,CV_BGR2GRAY);
      gray.convertTo(gray,CV_8UC1);
      
      cv::split(tmp,HSV_);  
      HSV_[2] = HSV_[0].mul(HSV_[1]);
      //cv::imshow("Mask",mask);      
      //cv::waitKey(0);
      
      /*cv::imshow("test",gray);
      cv::waitKey(0);*/
            
      cv_bridge::CvImage cvi;
      /*cvi.header.stamp = time;
      cvi.header.frame_id = "image";*/
      cvi.encoding = "mono8";
      HSV_[0].copyTo(cvi.image);
      cvi.toImageMsg(response.HImage);
      
      gray.copyTo(cvi.image);
      cvi.toImageMsg(response.grayImage);
      
      mask.copyTo(cvi.image);
      cvi.toImageMsg(response.mask);
      
      
      flag_service_ = false;
      
      image_sub_.unsubscribe();
      im_info_sub_.unsubscribe();
      stereo_pcl_sub_.unsubscribe();
  return true; 
}

// void Obj_selector::start(){
//   
//   ros::NodeHandle nh;
//   select_object_pub_ = nh.advertiseService("/ap/select_obect", &Obj_selector::selectObjectCb, this );  
// }

void Obj_selector::initialize(){
  
  
  // Read local parameters
  ros::NodeHandle nh_local("~");
  
  
  nh_local.param("WorldReference", world_frame_id_,std::string("/map"));
   
    
  select_object_pub_ = nh_.advertiseService("/ap/select_obect", &Obj_selector::selectObjectCb, this );  
  
  
  
}

int main(int argc, char** argv) {
  
  
  ros::init (argc, argv, "select_object");    
  
  
  Obj_selector obj_selector;
  obj_selector.initialize();  
  
  ros::MultiThreadedSpinner spinner(2);
  
  spinner.spin();
  
  
  
  return 0;
  
}