#ifndef DETECT_OBJECTS_XTION_H_
#define DETECT_OBJECTS_XTION_H_

//////// std includes ////////
#include <stdio.h>
#include <omp.h>

/////// ROS ///////
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/package.h>
#include <ros/time.h>


#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <laser_assembler/AssembleScans2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
/////// PCL ///////
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ros/conversions.h>


#include <pcl/common/time.h>

#include <pcl/features/normal_3d_omp.h>
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


#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/gicp.h> //generalized icp extension


#include <pr2_pick_perception/ObjectList.h>
#include <pr2_pick_perception/ObjectDetectionRequest.h>
#include <pr2_pick_perception/Object.h>
#include <pr2_pick_perception/LocalizeShelf.h>


struct Model
{
    int id; ///<model view id ... not really used anywhere but useful for debugging maybe
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; ///<model cloud from sensor point of view
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled; ///<downsampled cloud (used for first phase of pose estimate)
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
    void detect(const pcl::PointCloud< pcl::PointXYZ >::ConstPtr& cluster, int *matchedModelID, Eigen::Matrix4f *matchedTransform, Eigen::Matrix4f *matchedTransform2, const pcl::visualization::PCLVisualizer::Ptr &visu);
    
    ///callback for receiving detection commands
    bool detectCallback(pr2_pick_perception::LocalizeShelfRequest &request, pr2_pick_perception::LocalizeShelfResponse& response);
    
    
    ///\brief Returns model with id "id"
    void getModel(int id, Model *model);
    
    ///\brief Set max icp iterations
    void setMaxIter(int maxIter) {max_iter_ = maxIter;}
    
    ///\brief Set threshold for valid matches
    void setScoreThreshold(double thresh){score_thresh_ = thresh;}
    
  
     
private:
    ros::Publisher pub_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber pc_sub_;
    
    /// stereo to world transform 
    tf::StampedTransform stereo2world_;
    /// cloud to robot transform
    tf::StampedTransform cloud_to_robot_;
    /// robot to camera transform
    tf::StampedTransform robot_to_camera_;
    /// robot to world transform
    tf::StampedTransform robot_to_world_;
    /// robot to model transform
    tf::StampedTransform  cloud_to_model_;

    bool debug_;
    
    
    /// name of colored point cloud
    std::string topicLidar_;
    /// stores the object type the detector is detecting
    std::string obj_type_;
    /// robot refence system
    std::string robot_frame_id_;
    /// world reference system
    std::string world_frame_id_;
    /// stereo reference system
    std::string stereo_ref_system_;
    /// lidar reference system
    std::string cloud_frame_id_;
    /// camera reference system    
    std::string camera_frame_id_;
    /// model reference system
    std::string model_frame_id_;
    
    /// refence transform listener
    tf::TransformListener tf_;
    /// stereo transform
    //tf::TransformListener tf_stereo_;
    /// models for all the views of the object
    std::vector<Model> models_;
    /// cluster bounds for pre-filtering (min height, min width, max height, max width) 
    std::vector<double> cluster_bounds_; //pre-filter constraints for clusters to discard irrelevant clusters and speed up detection 
                                         //... get rid of this maybe when switching to our cvfh? Could still be useful for speed up purposes though.
                                         //We're not really expecting a lot of clutter in the two VRC scenarios though so maybe useless
    
    /// Image to send to the user
    std::vector<cv::Mat> HSV_;
    /// stereo PC params
    bool gotPCstereo_;
    bool image_ready_;
    bool pc_ready_;
    
    boost::mutex xtion_mtx_; 
    boost::mutex image_mtx_; 
    
    pcl::PointCloud<pcl::PointXYZ> xtionPC_;
    sensor_msgs::PointCloud2Ptr xtionPC2ptr_;
    ros::Time pc_timestamp_;
    
    image_geometry::PinholeCameraModel leftcamproj_; 
    
    sensor_msgs::ImageConstPtr l_image_msg_;    
    cv_bridge::CvImageConstPtr l_cv_ptr_;
    
    std::vector<cv::Point2f> roi_;
    cv::Size im_size_;
    
    /// Algorithm params
    double radius_search_;
    
    /// color segmentation
    bool color_segmentation_;
    double dist_threshold_;
    double point_color_threshold_;
    double RegionColorThreshold_;
    //double min_cluster_size_;
    /// plane segmentation threshold
    double PlanesegThres_;    
    int PlaneSize_;
    /// minimum height to detect objects
    double highplane_;
    
    /// hose segmentation mainly
    bool manual_segmentation_;
    
    ///pca alingment for non-fixed objects
    bool pca_alignment_;
   ///return only yaw + xyz
    bool only_yaw_;
   
    //window to show to the user
    const char* src_window_; 
   
    
    /// min number of points for a cluster to be considered
    int min_cluster_size_; 
    /// leaf size for voxel grid sampling of model and clusters
    double sample_size_;
    /// max distance tolerance during clustering ... needs to be larger for polaris due to large gaps when viewed from behind
    double cluster_tolerance_; 
    double score_thresh_; ///< threshold for selecting a valid match 
                          //TODO: add 2 thresholds for rough and fine match?
    int max_iter_; ///< max ICP iterations for model refinement
    bool on_table_; ///< flag whether object can be on a table ... mainly for optimization right now
    
    
    /// Transformation to the robot reference system
    //Eigen::Matrix4f cloud_to_robotE_;
    
    
    /// callback for stereo point cloud   
    void xtionPCcallback(const sensor_msgs::PointCloud2ConstPtr &pcloud);
    
    /// Image callback
    void processImage(const sensor_msgs::ImageConstPtr &l_image_msg, const sensor_msgs::CameraInfoConstPtr& l_info_msg);    
    
    ///extract clusters based on color   
    void extractClusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scene, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *clusters);
    
    
    
    /// synchronization 
    /// image subscriber
    image_transport::SubscriberFilter image_sub_;
    /// camera info subscriber   
    message_filters::Subscriber<sensor_msgs::CameraInfo> im_info_sub_;
    /// stereo point cloud subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> stereo_pcl_sub_;   
    
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;    

    
    /// for sync checking
    static void increment(int* value) {  ++(*value);  }
    /// checks for synchronized data streams, gets called every 15 seconds
    void checkInputsSynchronized();
    
    
    /// Countger synchronization variables
    int image_received_, im_info_received_, stereo_pcl_received_, all_received_;    
    // for sync checking
    ros::Timer check_synced_timer_;
    
     /// synchronization wrapper for data callback
    void dataCbSync(const sensor_msgs::ImageConstPtr &img_msg, 
                          const sensor_msgs::CameraInfoConstPtr &cam_info,                           
                          const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
    
    
    
    /// callback function for receiving image and stereo point cloud
    void dataCallback( const sensor_msgs::ImageConstPtr &img_msg, 
         const sensor_msgs::CameraInfoConstPtr &cam_info,                       
         const sensor_msgs::PointCloud2ConstPtr &pcl_msg);
    
     /// get the rigid transformation between two point clouds using svd decomposition
    void getTransformationFromCorrelation ( const Eigen::MatrixXd &source_pc, const Eigen::Vector4d & centroid_src,
        const Eigen::MatrixXd &target_pc, const Eigen::Vector4d & centroid_tgt, Eigen::Matrix4d &transformation_matrix);
    
};
#endif
