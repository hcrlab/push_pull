#include <multisense/rgb_pc.h>


void processImage(const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg) {
  
  leftcamproj.fromCameraInfo(l_info_msg);
  l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
  width = l_image_msg->width;
  height = l_image_msg->height;
  image_ready = true;
  camera_frame_id = l_info_msg->header.frame_id;
  static int cont = 0;
      cont++;
      
}



void callback(const sensor_msgs::PointCloud2ConstPtr & pc_msg) {
  try {   
    if (image_ready == false) {
      return;
    }
    static bool trans = false;
    cv::Mat image;
    
    l_cv_ptr->image.copyTo(image);
      
      lidar_frame_id = pc_msg->header.frame_id;
      static int cont = 0;
      cont++;
      tf::TransformListener tf_listener;
      std::string error_msg;
      
      if (trans == false){
        tf_listener.waitForTransform(camera_frame_id, lidar_frame_id, ros::Time(),ros::Duration(2.0), ros::Duration(0.0), &error_msg);
        tf_listener.lookupTransform(camera_frame_id,lidar_frame_id, ros::Time(), Tlidar2cam);      
        trans = true;
      }
      
     /*std::cout << lidar_frame_id.c_str()  << " to " << camera_frame_id.c_str()  << std::endl;
     for(int i=0;i<3;i++){
        std::cout << Tlidar2cam.getBasis()[i][0] << " ";
        std::cout << Tlidar2cam.getBasis()[i][1] << " ";
        std::cout << Tlidar2cam.getBasis()[i][2] << " " << Tlidar2cam.getOrigin()[i]	<<std::endl;
       
    }*/
      
      WPointCloud cloud;
      pcl::fromROSMsg (*pc_msg, cloud);   
      
      size_t n_points = cloud.points.size();
      rgbpc.points.resize(n_points);
      //copy the image      
      for (size_t i = 0; i < n_points; i++) {
        
	WPoint pt = cloud.points[i]; 
	
	//std::cout << "pt.x = " << pt.x << "," << "pt.y = " << pt.y << "pt.z = " << pt.z << std::endl;
        //initialize each point to black
        rgbpc.points[i].x = pt.x;
        rgbpc.points[i].y = pt.y;
        rgbpc.points[i].z = pt.z;
	
        uint32_t black = ((uint32_t)0 << 16 | (uint32_t)0 << 8 | (uint32_t)0);
        rgbpc.points[i].rgb = *reinterpret_cast<float*>(&black);       
      
       
	//tranform this point to the camera reference system
	tf::Point tfX(pt.x, pt.y, pt.z);	
	tf::Point tfXcam = Tlidar2cam * tfX;
	
	
	
	WPoint Xcam;	
	Xcam.x = tfXcam.m_floats[0];
	Xcam.y = tfXcam.m_floats[1];
	Xcam.z = tfXcam.m_floats[2];
	
	//std::cout << "Xcam.x = " << Xcam.x << ", Xcam.y = " << Xcam.y << ", Xcam.z = " << Xcam.z << std::endl;
	
	//find rgb value for this point in the image                  
	cv::Point3d xyz(Xcam.x, Xcam.y, Xcam.z); //a Point3d of the point in the pointcloud
	cv::Point2d im_point;  //will hold the 2d point in the image
	im_point = leftcamproj.project3dToPixel(xyz);
	
	//get rgb from 2d point
	cv::Vec3b intensity;
	uchar r,g,b;
	
	
	if (im_point.y < height &&  im_point.x < width && im_point.y >= 0 && im_point.x >= 0) {  	  
	  intensity = image.at<cv::Vec3b>((int)im_point.y,(int)im_point.x);
	  r = intensity.val[0];
          g = intensity.val[1];
          b = intensity.val[2];

	
	  //std::cout << "rgb = [" <<(int) r << "," <<(int)g<<"," << (int)b << "]" <<std::endl;
	  //give the new point the rgb value from the image and the original, untransformed x, y, z points
	  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	  rgbpc.points[i].rgb = *reinterpret_cast<float*>(&rgb);		      
	}
      
    	        
      }    
     
      
      sensor_msgs::PointCloud cloud_out1;
      sensor_msgs::PointCloud2 cloud_out2;
      
      pcl::toROSMsg(rgbpc,cloud_out2);
     
      sensor_msgs::convertPointCloud2ToPointCloud(cloud_out2,cloud_out1);     
        
      cloud_out1.header.stamp = pc_msg->header.stamp;
      cloud_out1.header.frame_id = pc_msg->header.frame_id;
      
      rgbpc_pub.publish(cloud_out1);
      
      
    }
    catch (tf::TransformException ex) {
      // only log tf error once every 20 times
      ROS_WARN_THROTTLE(20, "%s", ex.what());
      return;   // skip this packet
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
  rgbpc_pub = nh.advertise<sensor_msgs::PointCloud> ("hokuyo/rgb_pointcloud", 1);  
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topicLidar, 1, callback);
   
  //subscribe to image
  static int queue = 1; // ROS topic queue size
  image_transport::ImageTransport it(nh);
  
  std::string topicImleft = nh.resolveName("image");
  image_transport::CameraSubscriber Leftcam = it.subscribeCamera(topicImleft, queue, &processImage);

    
  ros::spin();
  
  return 0;
}

