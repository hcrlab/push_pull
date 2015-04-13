#include <multisense/crop_image.h>



void callbackmouse(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = cv::Point2f(x, y);
        drag = 1;
	
	std::cout << "the clicked point is = " << x << "," << y<< std::endl;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        cv::Mat img1 = frame.clone();
        point2 = cv::Point2f (x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        cv::imshow(src_window_, img1);
	//std::cout << "the dragged point is = " << x << "," << y<< std::endl;
	cv::waitKey(500);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        cv::Mat img2 = frame.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
	callback_im = true;
        cv::imshow(src_window_, img2);
	//cv::waitKey(0);
        
	std::cout << "the lift potin is = " << x << "," << y<< std::endl;
    }
    if (event ==CV_EVENT_RBUTTONDOWN){
      callback_im =false;
      select_flag = false;  
      
    }
}

void processImage(const sensor_msgs::ImageConstPtr &l_image_msg){//, const sensor_msgs::CameraInfoConstPtr& l_info_msg) {
  
  //leftcamproj.fromCameraInfo(l_info_msg);
  l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8);
  width = l_image_msg->width;
  height = l_image_msg->height;
  image_ready = true;
  //camera_frame_id = l_info_msg->header.frame_id;    
  l_cv_ptr->image.copyTo(frame);  
  
  cv::imshow(src_window_,frame);
  cv::waitKey(0);
  
   if (callback_im){
      message.region2d.top_left_x = point1.x/width; 
      message.region2d.top_left_y = point1.y/height; 
      message.region2d.bottom_right_x = point2.x/width;
      message.region2d.bottom_right_y = point2.y/height;
      message.obj_type = objt_name;
    }
    roi_pub.publish(message);
}



void pccallback(const sensor_msgs::PointCloud2ConstPtr & pc_msg) {
  if (image_ready == false) {
      return;
    }      
      
    lidar_frame_id = pc_msg->header.frame_id;   
    pcl::fromROSMsg (*pc_msg, pc_in);   
    pc_ready = true;
    
    pc_stamp = pc_msg->header.stamp;
  
}

// bool cropCallback( trooper_adaptive_perception_msgs::SelectObjectImg::Request &request, 
// 		  trooper_adaptive_perception_msgs::SelectObjectImg::Response &response) 
// {
//   try {   
//     if (image_ready == false)  {
//       return false;
//     }
//     
//    
//     
//     
//     if (callback_im){
//       response.region.region2d.top_left_x = point1.x; 
//       response.region.region2d.top_left_y = point1.y; 
//       response.region.region2d.bottom_right_x = point2.x;
//       response.region.region2d.bottom_right_y = point2.y;
//       response.region.obj_type = request.input.data;
//     }
//       
//    
//     return true;
//   }
//   catch (tf::TransformException ex) {
//     // only log tf error once every 20 times
//     ROS_WARN_THROTTLE(20, "%s", ex.what());
//     return false;   // skip this packet
//   }    
//   
//   
// }

int main(int argc, char** argv) {
  
  
  ros::init (argc, argv, "rgb_pc");    
  ros::NodeHandle nh;
  
  // Read local parameters
  ros::NodeHandle local_nh("~");

     
  local_nh.param("object_name",objt_name,std::string("vrc_valve"));

  std::string trigger_msg = nh.resolveName("trigger_message");
    
   roi_pub = nh.advertise<adaptive_perception_msgs::ObjectDetectionRequest> (trigger_msg, 1); //"/ap/run_object_detection"
   
  //subscribe to image
  static int queue = 1; // ROS topic queue size
  //image_transport::ImageTransport it(nh);
  
  std::string topicImleft = nh.resolveName("image");
  //image_transport::CameraSubscriber Leftcam = it.subscribeCamera(topicImleft, queue, &processImage);
  ros::Subscriber sub = nh.subscribe(topicImleft, 1, processImage);
  
  
  cv::namedWindow(src_window_);
  cv::setMouseCallback(src_window_,callbackmouse,0);
    
  ros::spin();
  
  return 0;
}

