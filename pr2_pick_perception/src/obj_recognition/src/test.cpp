#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/recognition/hv/greedy_verification.h>

#include <pcl/features/our_cvfh.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/filters/extract_indices.h>

int main(int argc, char **argv)
{    
    //all clouds and other helper classes (to keep the rest of the code free of cloud definitions)
    typedef pcl::PointXYZ PointT;
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvature(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pcl::ExtractIndices<PointT> extractor;
    pcl::ExtractIndices<pcl::Normal> extractor_normals;
    
//     pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourcvfh;
    
    
    //////////////////
    // 0. get model //
    //////////////////
//     std::string model_file;
//     pcl::console::parse_argument(argc, argv, "-model", model_file);
//     pcl::io::loadPCDFile(model_file, *model);
    
    ///////////////////
    // 1. get scene  //
    ///////////////////
    
    //load cloud from pcd
    std::string filename;
    pcl::console::parse_argument(argc, argv, "-scene", filename);
    pcl::io::loadPCDFile(filename, *cloud);

    //rotate point cloud to have proper orientation (FIXME: data)
    pcl::transformPointCloud(*cloud, *cloud, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));
//     pcl::transformPointCloud(*model, *model, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));

    //start visualizer containing point cloud
    pcl::visualization::PCLVisualizer vis("TR00P3R");
    pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> geom_handler(cloud);
    vis.addPointCloud(cloud, geom_handler, "scene");
//     pcl::visualization::PointCloudColorHandlerRandom<PointT> rand_handlerX(model);
//     vis.addPointCloud(model, rand_handlerX, "model");
    vis.resetCameraViewpoint();


    //////////////////////////////////////////
    // 2. filter cloud and estimate normals //
    //////////////////////////////////////////
    
    //TODO: maybe run MLS for use in plane segmentation to be able to use a lower threshold
    
    //filter cloud (statistical outlied removal)
    //     ROS_INFO_STREAM("cloud before filtering: " << *cloud); 
    //     pcl::StatisticalOutlierRemoval<PointT> sor;
    //     sor.setInputCloud(cloud);
    //     sor.setMeanK(50);
    //     sor.setStddevMulThresh(1.0);
    //     sor.filter(*cloud_filtered);
    //     ROS_INFO_STREAM("cloud after filtering: " << *cloud_filtered); 
    cloud_filtered = cloud;

    //smooth&upsample cloud using MLS
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_xyz);
//     printf("running MLS ...\n");
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_upsampled(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//     mls.setComputeNormals(true);
//     mls.setInputCloud(cloud_filtered_xyz);
//     mls.setSearchMethod(tree1);
//     mls.setSearchRadius(0.05);
//     mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
//     mls.setUpsamplingRadius(0.1);
//     mls.setUpsamplingStepSize(0.05);
//     mls.process(*cloud_upsampled);
//     printf("done\n");
    
    //estimate normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEst;
    normalEst.setSearchMethod(tree);
    normalEst.setInputCloud(cloud_filtered);
    normalEst.setKSearch(25);
    normalEst.compute(*cloud_normals);

//     vis.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, cloud_normals, 10, 0.02, "normals");

    //////////////////////////
    // 3. find ground plane //
    //////////////////////////

    //find biggest plane (=ground plane)
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
    sac.setAxis(Eigen::Vector3f(0,0,1)); //look for plane normal to z axis
    sac.setNormalDistanceWeight(0.1);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setMaxIterations(100);
    sac.setDistanceThreshold(0.05); //10cm threshold - required for segmenting the ground at ~20m due to noise, can use a lower threshold for manipulation task
    sac.setInputCloud(cloud_filtered);
    sac.setInputNormals(cloud_normals);
    sac.segment(*inliers_plane, *coefficients_plane);
    std::cout << "plane coefficients: " << *coefficients_plane << std::endl;
    std::cout << "inliers: " << inliers_plane->indices.size() << std::endl;

    //remove plane from point cloud
    extractor.setInputCloud(cloud_filtered);
    extractor.setIndices(inliers_plane);
    extractor.setNegative(true);
    
    pcl::PointCloud<PointT>::Ptr cloud_segmented(new pcl::PointCloud<PointT>);
    extractor.filter(*cloud_segmented);
    
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(cloud_segmented, 255, 0, 0);
    vis.addPointCloud(cloud_segmented, color_handler, "cloud_segmented");
    
    pcl::io::savePCDFile(boost::filesystem3::basename(filename)+"_no_ground.pcd", *cloud_segmented);
    
    //////////////////////
    // 4. find clusters //
    //////////////////////
    
    //extract clusters
    tree->setInputCloud(cloud_segmented);    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.50); //25cm
    ec.setMinClusterSize(150);
    ec.setMaxClusterSize(10000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_segmented);
    ec.extract(cluster_indices);
    
    printf("%d clusters found\n", cluster_indices.size());
    
    int j = 0;
    for(int i = 0; i < cluster_indices.size(); ++i)
    {
        printf("cluster has %d points\n", cluster_indices[i].indices.size()); 

        //extract cluster from point cloud
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        
        for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); ++pit)
            cloud_cluster->points.push_back(cloud_segmented->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        //display and save cluster
        std::stringstream cluster_name; 
        cluster_name << "cluster_" << i;
        
        pcl::visualization::PointCloudColorHandlerRandom<PointT> random_handler(cloud_cluster);
        vis.addPointCloud<PointT>(cloud_cluster, random_handler, cluster_name.str());
        
        pcl::io::savePCDFile(cluster_name.str()+".pcd", *cloud_cluster);
        
        
        /////////////////////////
        // 5. classify cluster //
        /////////////////////////
//         pcl::SampleConsensusInitialAlignment<PointT, PointT, >
    }

    vis.spin();
    
    return 0;
}
