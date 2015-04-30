#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pr2_pick_perception/shelf_cropping.h>
#include <pr2_pick_perception/Cluster.h>
#include <pr2_pick_perception/ClusterList.h>

CropShelf::CropShelf() {}

bool CropShelf::initialize() {
  ros::NodeHandle nh_local("~");
  ros::NodeHandle nh;

  // subscribe to point cloud topic
  std::string shelf_pose_topic = nh.resolveName("/shelf_pose_topic");
  // subscribe to shelf localization topic
  std::string pc_topic = nh.resolveName("/pc_topic");

  // shelf_pose_sub_ =
  // nh.subscribe<pr2_pick_perception::ObjectList>(shelf_pose_topic,10,&CropShelf::poseListener,this);
  pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
      pc_topic, 10, &CropShelf::pcCallBack, this);

  nh_local.param("RobotReference", robot_frame_id_,
                 std::string("/base_footprint"));
  nh_local.param("WorldReference", world_frame_id_, std::string("/map"));
  // nh_local.param("ModelReference",
  // model_frame_id_,std::string("/model_frame"));

  nh_local.param("ShelfReference", shelf_frame_id_,
                 std::string("/shelf_leftcorner_frame"));
  nh_local.param("Debug", debug_, false);

  nh_local.param("Width1", cell_width1_, 0.2671);
  nh_local.param("Width2", cell_width2_, 0.2991);
  nh_local.param("Height1", cell_height1_, 0.26);
  nh_local.param("Height2", cell_height2_, 0.23);
  nh_local.param("Depth", depth_cell_, 0.43);

  // Parameters for tweaking shelf crop
  nh_local.param("bottom_crop_offset", bottom_crop_offset_, 0.02);
  nh_local.param("top_crop_offset", top_crop_offset_, 0.02);
  nh_local.param("left_crop_offset", left_crop_offset_, 0.08);
  nh_local.param("right_crop_offset", right_crop_offset_, 0.02);
  nh_local.param("depth_close_crop_offset", depth_close_crop_offset_, 0.02);
  nh_local.param("depth_far_crop_offset", depth_far_crop_offset_, 0.02);
  nh_local.param("max_cluster_points", max_cluster_points_, 100000);
  nh_local.param("min_cluster_points", min_cluster_points_, 150);

  return true;
}

void CropShelf::pcCallBack(const sensor_msgs::PointCloud2::ConstPtr &pc_msg) {
  boost::mutex::scoped_lock lock(kinect_mtx_);
  cloud_frame_id_ = pc_msg->header.frame_id;
  pcl::fromROSMsg(*pc_msg, kinect_pc_);
  pc_timestamp_ = pc_msg->header.stamp;
  if (!pc_ready_) {
    ROS_INFO("Point cloud ready for crop.");
  }
  pc_ready_ = true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CropShelf::cropPC(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &shelf_pc, float width,
    float height, float depth, int cellID) {
  // total of cells in row 3
  int cellsrow = 3;

  //     int timesx = cellID % cellsrow;
  //     int timesy = cellID / cellsrow;
  //
  int timesx = 0;
  int timesy = 0;

  // float p1y = width * timesx;    float p1z = height * timesy;
  // float p2y = p1y + width;    float p2z = p1z + height;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_pc(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  printf("The number of points %d\n", (int)shelf_pc->points.size());
  // printf("point limits = [%f, %f, %f, %f, %f]\n", p1y,p2y,p1z,p2z,depth);
  for (int i = 0; i < shelf_pc->points.size(); i++) {
    pcl::PointXYZRGB point = shelf_pc->points[i];
    // The point cloud has been transformed around the frame of the bin.
    // The origin of the bin's frame is in the front bottom center of the bin.
    if (point.x < (depth - depth_far_crop_offset_) &&
        point.x >= (0 + depth_close_crop_offset_) &&
        point.y < (width / 2 - left_crop_offset_) &&
        point.y >= (-width / 2 + right_crop_offset_) &&
        point.z < (height - top_crop_offset_) &&
        point.z >= (0 + bottom_crop_offset_)) {
      cell_pc->push_back(point);
    }
  }
  ROS_INFO("Cropping PC");
  return cell_pc;
}

bool CropShelf::cropCallBack(pr2_pick_perception::CropShelfRequest &request,
                             pr2_pick_perception::CropShelfResponse &response) {
  pr2_pick_perception::Cell cell;
  std::string error_msg;

  if (!pc_ready_) {
    ROS_ERROR("No point cloud or no shelf detected\n");
    return false;
  } else {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_pc(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shelf_pc(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    if ("A" == request.cellID) {
      bin_frame_id_ = "bin_A";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);

      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height1_, depth_cell_, 0);
    }
    if ("B" == request.cellID) {
      bin_frame_id_ = "bin_B";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width2_, cell_height1_, depth_cell_, 1);
    }
    if ("C" == request.cellID) {
      bin_frame_id_ = "bin_C";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height1_, depth_cell_, 2);
    }
    if ("D" == request.cellID) {
      bin_frame_id_ = "bin_D";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height2_, depth_cell_, 3);
    }
    if ("E" == request.cellID) {
      bin_frame_id_ = "bin_E";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width2_, cell_height2_, depth_cell_, 4);
    }
    if ("F" == request.cellID) {
      bin_frame_id_ = "bin_F";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height2_, depth_cell_, 5);
    }
    if ("G" == request.cellID) {
      bin_frame_id_ = "bin_G";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height2_, depth_cell_, 6);
    }
    if ("H" == request.cellID) {
      bin_frame_id_ = "bin_H";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width2_, cell_height2_, depth_cell_, 7);
    }
    if ("I" == request.cellID) {
      bin_frame_id_ = "bin_I";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height2_, depth_cell_, 8);
    }
    if ("J" == request.cellID) {
      bin_frame_id_ = "bin_J";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_width1_, cell_height1_, depth_cell_, 9);
    }
    if ("K" == request.cellID) {
      bin_frame_id_ = "bin_K";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_height1_, cell_height1_, depth_cell_, 10);
    }
    if ("L" == request.cellID) {
      bin_frame_id_ = "bin_L";
      // depending on the cell we read that transform with respect to the pc
      // reference system
      if (tf_.canTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                           &error_msg)) {
        tf_.lookupTransform(bin_frame_id_, cloud_frame_id_, ros::Time(0),
                            cloud_to_bin_);
      } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cloud_frame_id_.c_str(), request.cellID.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_bin_.setIdentity();
      }
      pcl_ros::transformPointCloud(kinect_pc_, *shelf_pc, cloud_to_bin_);
      cell_pc = cropPC(shelf_pc, cell_height1_, cell_height1_, depth_cell_, 11);
    }
    // compute Euclidean clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;

    std::cout << "Final point cloud without planes size = "
              << cell_pc->points.size() << std::endl;
    // extract clusters EUclidean
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cell_pc);
    std::vector<pcl::PointIndices> clustersInd;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01);               // 25cm
    ec.setMinClusterSize(min_cluster_points_);  // need to go low for standpipe
    ec.setMaxClusterSize(max_cluster_points_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cell_pc);
    ec.extract(clustersInd);

    for (int i = 0; i < clustersInd.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointXYZRGB point;
      for (int j = 0; j < clustersInd[i].indices.size(); j++) {
        int index = clustersInd[i].indices[j];
        point.x = cell_pc->points[index].x;
        point.y = cell_pc->points[index].y;
        point.z = cell_pc->points[index].z;
        point.r = cell_pc->points[index].r;
        point.g = cell_pc->points[index].g;
        point.b = cell_pc->points[index].b;
        cluster->points.push_back(point);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }

    // pcl::visualization::PCLVisualizer::Ptr vis;
    // transform point cloud to robot_frame (where the point cloud is published)

    // if(debug_)
    //{
    //    vis.reset(new pcl::visualization::PCLVisualizer("Shelf Crooper --
    //    Debug"));
    //    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>
    //    scene_handler(shelf_pc, "x");//100, 100, 200);

    //    vis->addPointCloud(shelf_pc, scene_handler, "scene");
    //    vis->addCoordinateSystem();
    //    vis->setCameraPosition(-10,2,4,10,2,0,0,0,1);
    //    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
    //    cell_handler(cell_pc, 255, 255, 255.0);
    //    vis->addPointCloud(cell_pc, cell_handler, "Crooped cell");

    //    for(int i = 0; i < clusters.size(); ++i)
    //    {
    //        std::stringstream ss;
    //        ss << "cluster_raw_" << i;
    //        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB>
    //        cluster_handler(clusters[i]);
    //        vis->addPointCloud(clusters[i], cluster_handler, ss.str());
    //        vis->spinOnce();
    //    }
    //    vis->spin();
    //
    //}

    // copy the clusters to the object ObjectList
    pr2_pick_perception::ClusterList clusterlist;
    for (int i = 0; i < clusters.size(); i++) {
      pr2_pick_perception::Cluster cluster;
      cluster.header.frame_id = bin_frame_id_;
      cluster.header.stamp = pc_timestamp_;
      std::stringstream ss;
      ss << "cluster_" << i;
      cluster.id = ss.str();
      pcl::toROSMsg(*clusters[i], cluster.pointcloud);

      clusterlist.clusters.push_back(cluster);
    }
    if (clusters.size() == 0) {
      ROS_INFO("No clusters found. Returning whole cropped PC");
      pr2_pick_perception::Cluster cluster;
      cluster.header.frame_id = bin_frame_id_;
      cluster.header.stamp = pc_timestamp_;
      std::stringstream ss;
      ss << "cluster_0";
      cluster.id = ss.str();
      pcl::toROSMsg(*cell_pc, cluster.pointcloud);

      clusterlist.clusters.push_back(cluster);
    }
    response.locations = clusterlist;

    return true;
  }
}

/*
bool CropShelf::cropCallBack(pr2_pick_perception::CropShelfRequest &request,
                pr2_pick_perception::CropShelfResponse &response)
{

    pr2_pick_perception::Cell cell;
    std::string error_msg;

    if ( !pc_ready_)
    {
        ROS_ERROR("No point cloud or no shelf detected\n");
    }
    else
    {

        if (tf_.canTransform(robot_frame_id_, cloud_frame_id_, pc_timestamp_,
&error_msg)){
            tf_.lookupTransform(robot_frame_id_, cloud_frame_id_, pc_timestamp_,
cloud_to_robot_);
        }
        else
        {
            ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to
be available, " "will assume it as identity!",
                    cloud_frame_id_.c_str(),robot_frame_id_.c_str());
            ROS_WARN("Transform error: %s", error_msg.c_str());
            cloud_to_robot_.setIdentity();
            robot_frame_id_ = cloud_frame_id_;
        }


        if (tf_.canTransform(world_frame_id_, robot_frame_id_,pc_timestamp_,
&error_msg))
        {
            tf_.lookupTransform(world_frame_id_, robot_frame_id_, pc_timestamp_,
robot_to_world_);
        }
        else
        {
            ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to
be available, " "will assume it as identity!",
                    world_frame_id_.c_str(),robot_frame_id_.c_str());
            ROS_WARN("Transform error: %s", error_msg.c_str());
            robot_to_world_.setIdentity();
            world_frame_id_ = robot_frame_id_;
        }



        if (tf_.canTransform(shelf_frame_id_, model_frame_id_, ros::Time::now(),
&error_msg)){
            tf_.lookupTransform(shelf_frame_id_, model_frame_id_,
ros::Time::now(), shelf_to_origin_);
        }
        else
        {
            ROS_WARN_THROTTLE(10.0, "The tf from  '%s' to '%s' does not seem to
be available, " "will assume it as identity!",
                    model_frame_id_.c_str(),shelf_frame_id_.c_str());
            ROS_WARN("Transform error: %s", error_msg.c_str());
            shelf_to_origin_.setIdentity();
        }


        pcl::visualization::PCLVisualizer::Ptr vis;
        //transform point cloud to robot_frame (where the point cloud is
published)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr shelf_pc(new
pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_ros::transformPointCloud(kinect_pc_,*shelf_pc,cloud_to_robot_);

        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>
scene_handler(shelf_pc, "x");//100, 100, 200);
        if (debug_) {
            vis.reset(new pcl::visualization::PCLVisualizer("Shelf Crooper --
Debug"));
            vis->addPointCloud(shelf_pc, scene_handler, "scene");
            vis->addCoordinateSystem();
            vis->setCameraPosition(-10,2,4,10,2,0,0,0,1);
            vis->spin();
        }

        //move point cloud to shelf reference system
        pcl_ros::transformPointCloud(*shelf_pc,*shelf_pc,shelf_transform_.inverse());

//         if (debug_) {
//            // vis->removePointCloud("scene");
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
cell_handler(shelf_pc, 255, 255, 255.0);
//             vis->addPointCloud(shelf_pc, cell_handler, request.cellID);
//
//             vis->spin();
//         }

     // move the origin of the shelf to the left top corner
       // pcl::PointCloud<pcl::PointXYZRGB>::Ptr shelf_origin(new
pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_ros::transformPointCloud(*shelf_pc,*shelf_pc,shelf_to_origin_.inverse());

//         if (debug_) {
//             vis->removePointCloud(request.cellID);
// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
cell_handler(shelf_pc, 255, 0, 255.0);
//             vis->addPointCloud(shelf_pc, cell_handler, request.cellID);
//
//             vis->spin();
//         }



        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cell_pc(new
pcl::PointCloud<pcl::PointXYZRGB>);

        if ( "A" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height1_,
depth_cell_,0);
        if ( "B" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width2_,cell_height1_,
depth_cell_,1);
        if ( "C" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height1_,
depth_cell_,2);
        if ( "D" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height2_,
depth_cell_,3);
        if ( "E" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width2_,cell_height2_,
depth_cell_,4);
        if ( "F" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height2_,
depth_cell_,5);
        if ( "G" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height2_,
depth_cell_,6);
        if ( "H" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width2_,cell_height2_,
depth_cell_,7);
        if ( "I" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height2_,
depth_cell_,8);
        if ( "J" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_width1_,cell_height1_,
depth_cell_,9);
        if ( "K" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_height1_,cell_height1_,
depth_cell_,10);
        if ( "L" == request.cellID)
            cell_pc = cropPC(shelf_pc,cell_height1_,cell_height1_,
depth_cell_,11);

        //transform point cloud to robot_frame (where the point cloud is
published)
        pcl_ros::transformPointCloud(*cell_pc,*cell_pc,shelf_to_origin_);
        //move point cloud to shelf reference system
        pcl_ros::transformPointCloud(*cell_pc,*cell_pc,shelf_transform_);

        //compute Euclidean clusters
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;


        std::cout << "Final point cloud without planes size = " <<
cell_pc->points.size() << std::endl;
        //extract clusters EUclidean
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new
pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cell_pc);
        std::vector<pcl::PointIndices> clustersInd;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.01); //25cm
        ec.setMinClusterSize(150); //need to go low for standpipe
        ec.setMaxClusterSize(100000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cell_pc);
        ec.extract(clustersInd);


        for(int i = 0; i < clustersInd.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new
pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointXYZRGB point;
            for(int j=0; j < clustersInd[i].indices.size();j++){
                int index = clustersInd[i].indices[j];
                point.x = cell_pc->points[index].x;
                point.y = cell_pc->points[index].y;
                point.z = cell_pc->points[index].z;
                cluster->points.push_back(point);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        if(debug_)
        {

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
cell_handler(cell_pc, 255, 255, 255.0);
            vis->addPointCloud(cell_pc, cell_handler, "Crooped cell");

            vis->removePointCloud("scene");
            vis->removePointCloud(request.cellID);

            for(int i = 0; i < clusters.size(); ++i)
            {
                std::stringstream ss;
                ss << "cluster_raw_" << i;
                pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB>
cluster_handler(clusters[i]);
                vis->addPointCloud(clusters[i], cluster_handler, ss.str());
                vis->spinOnce();
            }
            vis->spin();

        }

        // copy the clusters to the object ObjectList
        pr2_pick_perception::ClusterList clusterlist;
        for (int i=0; i < clusters.size(); i++)
        {

            pr2_pick_perception::Cluster cluster;
            cluster.header.frame_id = model_frame_id_;
            cluster.header.stamp = pc_timestamp_;
            std::stringstream ss;
            ss << "cluster_" << i;
            cluster.id = ss.str();
            pcl::toROSMsg(*clusters[i],cluster.pointcloud);

            clusterlist.clusters.push_back(cluster);
        }

       response.locations = clusterlist;

    }


}*/

// void
// CropShelf::poseListener(pr2_pick_perception::ObjectList shelfdetection)
// {
//     if(shelfdetection.objects.size() > 0 )
//     {
//         shelf_pose_ = true;
//         shelf_transform_.setOrigin(tf::Vector3(shelfdetection.objects[0].pose.position.x,
//         shelfdetection.objects[0].pose.position.y,
//                                                shelfdetection.objects[0].pose.position.z));
//         shelf_transform_.setRotation(tf::Quaternion(
//         shelfdetection.objects[0].pose.orientation.x,
//         shelfdetection.objects[0].pose.orientation.y,
//                                                      shelfdetection.objects[0].pose.orientation.z,
//                                                      shelfdetection.objects[0].pose.orientation.w));
//
//         model_frame_id_ = shelfdetection.objects[0].header.frame_id;
//     }
//     else
//     {
//         ROS_ERROR("Shelf pose not available");
//     }
//
//
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "shelf_cropper");

  ros::NodeHandle nh;

  CropShelf cropper;

  if (!cropper.initialize()) {
    ROS_FATAL("Shelf cropper initialization failed. Shutting down node.");
    return 1;
  }

  ros::ServiceServer server(
      nh.advertiseService("shelf_cropper", &CropShelf::cropCallBack, &cropper));

  ros::spin();

  return 0;
}
