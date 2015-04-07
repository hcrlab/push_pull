#include <multisense/crop_pc.h>

void callbackmouse(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = cv::Point(x, y);
        drag = 1;

	std::cout << "the clicke potin is = " << x << "," << y<< std::endl;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        cv::Mat img1 = frame.clone();
        point2 = cv::Point(x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        cv::imshow(src_window, img1);
	//std::cout << "the dragged point is = " << x << "," << y<< std::endl;
	cv::waitKey(4);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        cv::Mat img2 = frame.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
	callback_im = true;
        cv::imshow(src_window, img2);
	//cv::waitKey(0);

	std::cout << "the lift potin is = " << x << "," << y<< std::endl;
    }
    if (event ==CV_EVENT_RBUTTONDOWN){
      callback_im =false;
      select_flag = false;

    }
}

void processImage(const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg) {

  leftcamproj.fromCameraInfo(l_info_msg);
  l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::BGR8);
  width = l_image_msg->width;
  height = l_image_msg->height;
  image_ready = true;
  camera_frame_id = l_info_msg->header.frame_id;
  l_cv_ptr->image.copyTo(frame);
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

bool cropCallback( adaptive_perception_msgs::GetPointCloudROI::Request &request,
		  adaptive_perception_msgs::GetPointCloudROI::Response &response)
{
  try {
    if (image_ready == false || pc_ready == false)  {
      return false;
    }
    cv::Rect roi;
    roi = cv::Rect( request.region.top_left_x*width,request.region.top_left_y*height,
		   (request.region.bottom_right_x - request.region.top_left_x)*width ,
		   ( request.region.bottom_right_y -request.region.top_left_y)* height);


    tf::TransformListener tf_listener;
    std::string error_msg;

    tf_listener.waitForTransform(camera_frame_id, lidar_frame_id, ros::Time(),ros::Duration(10.0), ros::Duration(0.0), &error_msg);
    tf_listener.lookupTransform(camera_frame_id,lidar_frame_id, ros::Time(), Tlidar2cam);


    size_t n_points = pc_in.points.size();
    pc_crop.clear();
    //copy the image
    for (size_t i = 0; i < n_points; i++) {

      //tranform this point to the camera reference system
      tf::Point tfX(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      tf::Point tfXcam = Tlidar2cam * tfX;

      WPoint Xcam ( tfXcam.m_floats[0], tfXcam.m_floats[1], tfXcam.m_floats[2]);

      //find rgb value for this point in the image
      cv::Point3d xyz(Xcam.x, Xcam.y, Xcam.z); //a Point3d of the point in the pointcloud
      cv::Point2d im_point;  //will hold the 2d point in the image
      im_point = leftcamproj.project3dToPixel(xyz);


      if (roi.contains(im_point))
	pc_crop.push_back(pc_in.points[i]);

    }

    sensor_msgs::PointCloud2 cloud_out;

    pcl::toROSMsg(pc_crop,response.cloud);

    response.cloud.header.stamp = pc_stamp;
    response.cloud.header.frame_id = lidar_frame_id;

    //croppc_pub.publish(cloud_out);
    //response.cloud.data.assign(cloud_out.data.begin(),cloud_out.data.end()); ;
    return true;
  }
  catch (tf::TransformException ex) {
    // only log tf error once every 20 times
    ROS_WARN_THROTTLE(20, "%s", ex.what());
    return false;   // skip this packet
  }


}

int main(int argc, char** argv) {


  ros::init (argc, argv, "rgb_pc");
  ros::NodeHandle nh;

  // Read local parameters
  ros::NodeHandle local_nh("~");

  std::string laser_frame_id;
  std::string camera_frame_id;

  std::string topicLidar = nh.resolveName("/lidar");
  croppc_pub = nh.advertiseService("/ap/get_point_cloud_roi", cropCallback );
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topicLidar, 1, pccallback);

  //subscribe to image
  static int queue = 1; // ROS topic queue size
  image_transport::ImageTransport it(nh);

  std::string topicImleft = nh.resolveName("image");
  image_transport::CameraSubscriber Leftcam = it.subscribeCamera(topicImleft, queue, &processImage);




  ros::spin();

  return 0;
}

