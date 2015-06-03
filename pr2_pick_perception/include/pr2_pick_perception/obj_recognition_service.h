///Service 
#include <pr2_pick_perception/MatchCluster.h>


#include <ros/ros.h>
#include <string>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <pcl/visualization/pcl_visualizer.h>


// TF
#include <tf/transform_listener.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>


#ifndef _PR2_PICK_PERCEPTION_MATCH_CLUSTER_SERVICE_H_
#define _PR2_PICK_PERCEPTION_MATCH_CLUSTER_SERVICE_H_

namespace pr2_pick_perception {
    
class MatchDescriptor {
    
public: 
    
    MatchDescriptor();
    ~MatchDescriptor(){};
    
     ///\brief initialize object matcher by reading parameters and loading descriptors
    ///\return false if loading of descriptors failed or parameters wrong/missing, true otherwise
    bool initialize();
    
    bool matchCallback(MatchCluster::Request& request,
                        MatchCluster::Response& response);

    
private:
    std::string descriptor_;
    std::string descriptors_dir_;
    int min_matches_;
    
   
    /// camera reference system    
    std::string camera_frame_id_, depth_camera_frame_id_;
    
    std::string matchertype_;
     /// refence transform listener
    tf::TransformListener tf_;
     /// cloud to camera transform
    tf::StampedTransform cloud_to_camera_;
    //image size
    cv::Size im_size_, depth_im_size_;
    
    bool debug_;
    cv::Ptr <cv::DescriptorMatcher > matcher_;
    
    boost::mutex img_mtx_,img_depth_mtx_ ; 
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    
    //image topic Subscriber
    image_transport::CameraSubscriber cam_sub_,depth_sub_;
    
    //flag to indicate image has been read
    bool image_ready_;
    
    bool depth_ready_;
    
    ///descriptor_ distance
    double descr_distance_;
    
    /// sensor error
    double sensor_error_;
    
    //RANSAC iterations
    int ransac_iterations_;
    cv::Mat image_;
    cv::Mat depth_,depth_visible_;
    
    void createVisualizer ();
   
    void viewPC(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& pc, const std::string &name );
    
    cv::Mat K_;
     //original point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cluster_pc_;
    
    ///\brief compute the mask used to compute descriptors in the color image
    void computeMask(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcloud, cv::Mat *image, cv::Mat *mask);
    
    void compute_pose(const std::vector<cv::KeyPoint> &keypoints, const cv::Mat &matches_3d, float span,cv::Mat *R, cv::Mat *T);
    
    void get3Dpoints( const std::vector<cv::Point>  &keypointsMSER,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcloud, 
                  pcl::PointCloud<pcl::PointXYZ>::Ptr points3D);
    
    void getsubmask(const cv::Mat &mask, const std::vector<cv::Point>  &keypointsMSER, cv::Mat *submask);
    
    void computeDescriptors(const cv::Mat &image,const cv::Mat &mask, std::vector<std::vector<cv::Point> > &contours, 
                            cv::Mat *descriptors, int histSize, std::vector<cv::Mat> *submasks);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcluster(const cv::Mat &mask, const pcl::PointCloud< pcl::PointXYZRGB >::ConstPtr& pcloud);
    };
};  // namespace pr2_pick_perception

#endif  // _PR2_PICK_PERCEPTION_ITEM_DESCRIPTOR_SERVICE_H_
