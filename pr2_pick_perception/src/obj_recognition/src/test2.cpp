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
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h> //generalized icp extension
#include <pcl/registration/icp_nl.h> //Levenberg-Marquardt icp 
#include <pcl/common/pca.h>


bool loadModel(const std::string &filename, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *models);

int main(int argc, char **argv)
{    
    
//     const MAX_X = 
    
    int max_iter = 100;
    pcl::console::parse_argument(argc, argv, "-iter", max_iter);
    
    //all clouds and other helper classes (to keep the rest of the code free of cloud definitions)
    typedef pcl::PointXYZ PointT;
    std::vector<pcl::PointCloud<PointT>::Ptr> models;
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    
    //////////////////
    // 0. get model //
    //////////////////
    std::vector<int> idxUnusedVar;
    std::string model_file;
    pcl::console::parse_argument(argc, argv, "-model", model_file);
    models.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));
    pcl::io::loadPCDFile(model_file, *models.back());
    pcl::removeNaNFromPointCloud(*models.back(), *models.back(), idxUnusedVar);
    
    std::string model_file2;
    pcl::console::parse_argument(argc, argv, "-model2", model_file2);

    
    //////////////////////
    // 1. get cluster  //
    /////////////////////
    
    //load cloud from pcd
    std::string filename;
    pcl::console::parse_argument(argc, argv, "-scene", filename);
    pcl::io::loadPCDFile(filename, *cluster);
    pcl::removeNaNFromPointCloud(*cluster, *cluster, idxUnusedVar);

    //rotate point cloud to have proper orientation (FIXME: data)
//     pcl::transformPointCloud(*cloud, *cloud, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));
//     pcl::transformPointCloud(*model, *model, pcl::getTransformation(0,0,0,0,90*M_PI/180.0,90*M_PI/180.0));

    //start visualizer containing point cloud
    pcl::visualization::PCLVisualizer vis("TR00P3R");
    pcl::visualization::PointCloudGeometryHandlerXYZ<PointT> geom_handler(cluster);
    vis.addPointCloud(cluster, geom_handler, "scene");
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> model_handler(model, 0, 0, 200);
//     vis.addPointCloud(model, model_handler, "model");
    vis.resetCameraViewpoint();

    //////////////////////////////////////
    // 2. filter clusters based on size //
    //////////////////////////////////////
//     Eigen::Vector4f centroid;
//     Eigen::Vector4f minPt, maxPt;
//     pcl::compute3DCentroid(*cluster, centroid);
//     pcl::getMinMax3D(*cluster, minPt, maxPt);
//     
//     pcl::PCA<PointT> pca;
//     pca.setInputCloud(cluster);
//     Eigen::Quaternionf orientation;
//     
//     Eigen::Vector3f centr(centroid[0], centroid[1], centroid[2]);
//     vis.addCube(centr, orientation, maxPt[0]-minPt[0], maxPt[1]-minPt[1], maxPt[2]-minPt[2]);
    
    //run thread for each model to speed up process - correct model should return after ~5-10ms 
    //TODO: optimize, use our-cvfh to prune possible models, try to stop other threads once one of them found a match
    int numThreads = models.size();
    bool finished = false;
    #pragma omp parallel for num_threads(numThreads) schedule(dynamic, models.size()/numThreads)
    for(int modelIdx = 0; modelIdx < models.size(); ++modelIdx)
    {
        pcl::PointCloud<PointT>::Ptr model = models[modelIdx];

        
        ///////////////////////////////////////
        // 3. Shift clouds based on centroid //
        ///////////////////////////////////////
        //TODO: maybe add eigen vectors (orientation)?
        Eigen::Vector4f centModel, centCluster;
        pcl::compute3DCentroid(*model, centModel);
        pcl::compute3DCentroid(*cluster, centCluster);
        Eigen::Vector4f trans = centCluster - centModel;
        
        pcl::PointCloud<PointT>::Ptr model_transformed(new pcl::PointCloud<PointT>);
        Eigen::Quaternionf rot; //TODO: maybe add eigen vectors (orientation)?
        pcl::transformPointCloud(*model, *model_transformed, Eigen::Vector3f(trans[0],trans[1],trans[2]), rot);
        
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_modelTrans(model_transformed, 200, 200, 0);
        vis.addPointCloud(model_transformed, color_handler_modelTrans, "model_transformed");
        
        ////////////////////////
        // 4. register clouds //
        ////////////////////////
        pcl::PointCloud<PointT>::Ptr modelRegistered(new pcl::PointCloud<PointT>);
        
        pcl::ScopeTime t("ICP...");
        
//         pcl::Registration<PointT, PointT>::Ptr registration (new pcl::IterativeClosestPoint<PointT, PointT>);
//         pcl::Registration<PointT, PointT>::Ptr registration(new pcl::IterativeClosestPointNonLinear<PointT, PointT>);
        pcl::Registration<PointT, PointT>::Ptr registration(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>);
        registration->setInputSource(model_transformed);
        registration->setInputTarget(cluster);
        registration->setMaxCorrespondenceDistance(0.1);
        registration->setRANSACOutlierRejectionThreshold(0.1);
        registration->setTransformationEpsilon(0.000001);
        registration->setMaximumIterations(max_iter);
        
        registration->align(*modelRegistered);
        Eigen::Matrix4f transformation_matrix = registration->getFinalTransformation();
        
        bool sendResult = false;
        #pragma omp critical
        {
            if(!finished && registration->hasConverged())
            {
                finished = true;
                sendResult = true;
            }
            else
            {
                if(registration->hasConverged())
                    printf("Other thread was faster, but I converged too!\n");
                else
                    printf("other thread was faster and I didn't converge :(\n");
            }
        }
        
        if(sendResult)
        {
            printf("=== Results for model %d (%fs):\n", modelIdx, t.getTimeSeconds());
            printf("ICP fitness score (model %d): %f", modelIdx, registration->getFitnessScore());
            cout << "Registered transform: " << transformation_matrix << endl;
            
            std::stringstream cms;
            cms << "modelReg" << modelIdx;
            pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler_modelReg(modelRegistered, 255, 0, 0);
            vis.addPointCloud(modelRegistered, color_handler_modelReg, cms.str());   
        }
    }


    vis.spin();
    
    return 0;
}

bool loadModel(const std::string& filename, std::vector< boost::shared_ptr< pcl::PointCloud< pcl::PointXYZ > > >* models)
{
    models->push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));
    pcl::io::loadPCDFile(filename, *models.back());
    
    //filter NaNs
    std::vector<int> idxUnusedVar;
    pcl::removeNaNFromPointCloud(*models.back(), *models.back(), idxUnusedVar);
    
    //downsize 
}
