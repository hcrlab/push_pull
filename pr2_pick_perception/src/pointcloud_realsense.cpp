#include <pr2_pick_perception/pointcloud_realsense.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>

namespace pr2_pick_perception {
    
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudRealsense::colorPC(cv::Mat depth, cv::Mat color)
{
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>); 
    
    for (std::size_t i=0; i < depth.rows; i++)
    {        
        for (std::size_t j=0; j < depth.cols; j++)
        {
            if (depth.at<float>(i,j)>0.0)
            {
                cv::Mat res;
                pcl::PointXYZRGB pcolor;
                cv::Point3d query_point;
                query_point = depthproj_.projectPixelTo3dRay(cv::Point2d(j,i));
                query_point *= depth.at<float>(i,j)/1000.;
                
                //transform the point into color reference sytem
                res = R_ *  cv::Mat( query_point, false) +  T_ ;
                res.copyTo(cv::Mat ( query_point, false));
                
                pcolor.x = query_point.x;
                pcolor.y = query_point.y;
                pcolor.z = query_point.z;

                cv::Point2d im_point;  //will hold the 2d point in the image
                im_point = colorproj_.project3dToPixel(query_point);
                int u,v;
                
                v = (int) (im_point.x + .5);
                u = (int) (im_point.y + .5);
                
                if (u <= color.rows && u >=1  && v <= color.cols && v >=1)
                {
                    uchar pr, pg, pb;

                    //Get RGB info
                    pb = color.at<cv::Vec3b>(u,v)[0];                
                    pg = color.at<cv::Vec3b>(u,v)[1];
                    pr = color.at<cv::Vec3b>(u,v)[2];
                    
                                
                    uint32_t rgb = (static_cast<uint32_t>(pr) << 16 | static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                    pcolor.rgb = *reinterpret_cast<float*>(&rgb);
                    pcColor->push_back(pcolor);
                }
                        
            }
        }   
    }
    return pcColor;
    
}

void PointCloudRealsense::pointcloudCallBack(const sensor_msgs::ImageConstPtr& msgcolor, const sensor_msgs::ImageConstPtr& msgdepth)
{
        /// compute the points cloud
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msgdepth, sensor_msgs::image_encodings::TYPE_32FC1);  
    cv::Mat depth, color;
    cv_ptr->image.copyTo(depth);
    //undistort depth
    cv::Mat depth_rect;
    cv::remap(depth,depth_rect,map_depth_x_,map_depth_y_,cv::INTER_NEAREST);
    cv::imshow("depth", depth);
    cv::imshow("rectified depth", depth_rect);
    cvWaitKey(25);
    
    cv_ptr = cv_bridge::toCvCopy(msgcolor, sensor_msgs::image_encodings::BGR8);  
    cv_ptr->image.copyTo(color);
    
    //get the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcColor (new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcColor = colorPC(depth_rect,color);
    //to ros msg
    sensor_msgs::PointCloud2 pccolor;
    pcl::toROSMsg(*pcColor,pccolor);
    pccolor.header.stamp = msgcolor->header.stamp;
    pccolor.header.frame_id = msgcolor->header.frame_id;
    
    pc_pub_.publish(pccolor);
   
}



bool PointCloudRealsense::init()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");
    
    R_ = cv::Mat::eye(3,3,CV_64F);
    Kdepth_ = cv::Mat::eye(3,3,CV_64F);
    Kcolor_ = cv::Mat::eye(3,3,CV_64F);
    T_ = cv::Mat::zeros(3,1,CV_64F);
    
           
    std::string img_topic = nh.resolveName("/image_topic");   
    std::string depth_topic = nh.resolveName("/depth_topic");
    
    image_transport::ImageTransport it(nh);
    image_transport::ImageTransport it_depth(nh);
    
    im_color_sub_.subscribe(it, img_topic, 5); 
    im_depth_sub_.subscribe(it_depth, depth_topic, 5); 
    
  
    sync_.reset(new Async( ASyncPolicy(10), im_color_sub_, im_depth_sub_));
    sync_->registerCallback(boost::bind(&PointCloudRealsense::pointcloudCallBack,this, _1, _2));
    
    std::string color_info_topic = nh.resolveName("/color_info");
    std::string depth_info_topic = nh.resolveName("/depth_info");
    
    sensor_msgs::CameraInfoConstPtr color_info;
    color_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(color_info_topic);//,nh,ros::Duration(15.));
    colorproj_.fromCameraInfo(color_info);     
    color_frame_id_ = color_info->header.frame_id;
    color_size_.height = color_info->height;
    color_size_.width = color_info->width; 
    
    sensor_msgs::CameraInfoConstPtr depth_info;
    depth_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(depth_info_topic);//,nh,ros::Duration(15.));
    depthproj_.fromCameraInfo(depth_info);     
    depth_frame_id_ = depth_info->header.frame_id;
    depth_size_.height = depth_info->height;
    depth_size_.width = depth_info->width; 
    
    for (int i=0; i < 9 ; i++)
    {
        Kcolor_.at<double>(i) = color_info->K[i];
        Kdepth_.at<double>(i) = depth_info->K[i];
        R_.at<double>(i) = color_info->R[i];
    }
    
    int size_coeff_depth = depth_info->D.size();
    cv::Mat coeffDepth = cv::Mat::zeros(size_coeff_depth,1,CV_64F);
    for (int i=0; i < size_coeff_depth; i++)
        coeffDepth.at<double>(i) = depth_info->D[i];
    
    T_.at<double>(0) = color_info->P[3];   
    T_.at<double>(1) = color_info->P[7];   
    T_.at<double>(2) = color_info->P[11];   
    
    

    cv::initUndistortRectifyMap(Kdepth_,coeffDepth,cv::Mat(),Kdepth_,depth_size_,CV_32FC1,map_depth_x_,map_depth_y_);
    std::cout << "R = " << R_ << std::endl << "T " << T_ << std::endl;
    
    
    std::string pc_topic = nh.resolveName("/pc_realsense");
    pc_pub_ =   nh.advertise<sensor_msgs::PointCloud2>(pc_topic, 1, true);     
   

    return true;
}
};


int main(int argc, char **argv)
{    
    ros::init(argc, argv, "pointcloud_realsense");

    pr2_pick_perception::PointCloudRealsense pcreal;    
    
    if(!pcreal.init())
    {
        ROS_FATAL("Point cloud publisher RealSense initialization failed. Shutting down node.");
        return 1;
    }
    ros::spin();
    
    return 0;
}
