#include <pr2_pick_perception/obj_recognition_service.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_subscriber.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <visualization_msgs/Marker.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/nonfree/nonfree.hpp>

//#include <opencv_candidate/lsh.h>
#include <pcl/filters/filter.h>

#include <sensor_msgs/image_encodings.h>

//#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <boost/concept_check.hpp>

#include <vtkTransform.h>
#include <ros/package.h>

namespace pr2_pick_perception {

MatchDescriptor::MatchDescriptor() {}

bool MatchDescriptor::initialize() {
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    // descriptor type, ORB, SIFT

    nh_local.param("descriptor_dir", descriptors_dir_, std::string(""));

    nh_local.param("descriptor_distance", descr_distance_, 0.5);

    // min number of matches to succeed
    nh_local.param("min_matches", min_matches_, 5);

    // min number of matches to succeed
    nh_local.param("debug", debug_, false);

    // ransca iterations
    nh_local.param("ransac_iterations", ransac_iterations_, 15);

    nh_local.param("sensor_error", sensor_error_, 0.1);

    std::string img_topic = nh.resolveName("/image_topic");

    image_transport::ImageTransport it(nh);

    im_size_.height = 480;
    im_size_.width = 640;

    camera_frame_id_ = "/head_mount_kinect_rgb_optical_frame";
    K_ = cv::Mat::eye(3, 3, CV_64F);
    K_.at<double>(0, 0) = 525.0;
    K_.at<double>(0, 2) = 319.5;
    K_.at<double>(1, 1) = 525.0;
    K_.at<double>(1, 2) = 239.5;

    if (debug_) {
        // createVisualizer();
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatchDescriptor::subcluster(
    const cv::Mat &mask,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcluster(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Point3d point;

    cv::Mat res;

    pcl::PointXYZRGB pcolor;

    for (std::size_t i = 0; i < pcloud->points.size(); i++) {
        cv::Point3d point,
            xyz(pcloud->points[i].x, pcloud->points[i].y,
                pcloud->points[i]
                    .z);  // a Point3d of the point in the pointcloud

        cv::Mat res;
        res = K_ * cv::Mat(xyz, false);
        res.copyTo(cv::Mat(point, false));
        // std::cout <<  point << std::endl;

        int v, u;
        v = (int)(point.x / point.z + .5);
        u = (int)(point.y / point.z + .5);

        if (u <= mask.rows && u >= 1 && v <= mask.cols && v >= 1)
            if (xyz.z < 1.0)
                if (mask.at<uchar>(u, v) > 0)
                    subcluster->push_back(pcloud->points[i]);
    }
    return subcluster;
}

void MatchDescriptor::computeMask(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcloud, cv::Mat *image,
    cv::Mat *mask) {
    // project the points to the image
    *image = cv::Mat::zeros(im_size_, CV_8UC3);
    *mask = cv::Mat::zeros(im_size_, CV_8U);

    //  std::cout << "K " << K << std::endl;
    for (std::size_t i = 0; i < pcloud->points.size(); i++) {
        cv::Point3d point,
            xyz(pcloud->points[i].x, pcloud->points[i].y,
                pcloud->points[i]
                    .z);  // a Point3d of the point in the pointcloud

        cv::Mat res;
        res = K_ * cv::Mat(xyz, false);
        res.copyTo(cv::Mat(point, false));
        // std::cout <<  point << std::endl;

        int v, u;
        v = (int)(point.x / point.z + .5);
        u = (int)(point.y / point.z + .5);

        if (u <= image->rows && u >= 1 && v <= image->cols && v >= 1) {
            if (xyz.z < 1.0) {
                // std::cout << ij << std::endl;
                pcl::PointXYZRGB p = pcloud->points[i];

                image->at<cv::Vec3b>(u, v)[0] = p.r;
                image->at<cv::Vec3b>(u, v)[1] = p.g;
                image->at<cv::Vec3b>(u, v)[2] = p.b;

                mask->at<uchar>(u, v) = 1;
            }
        }
    }

    // cv::dilate(*mask, *mask, cv::Mat(), cv::Point(-1, -1), 1);
}

void MatchDescriptor::viewPC(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc,
                             const std::string &name) {
    if (debug_) {
        viewer_->removeAllPointClouds();
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>
            cluster_handler(pc);
        viewer_->addPointCloud(pc, cluster_handler, name.c_str());

        viewer_->spin();
    }
}

double computeProduct(cv::Point p, cv::Point2f a, cv::Point2f b) {
    double k = (a.y - b.y) / (a.x - b.x);
    double j = a.y - k * a.x;
    return k * p.x - p.y + j;
}

bool isInROI(cv::Point p, cv::Point2f roi[]) {
    double pro[4];
    for (int i = 0; i < 4; ++i) {
        pro[i] = computeProduct(p, roi[i], roi[(i + 1) % 4]);
    }
    if (pro[0] * pro[2] < 0 && pro[1] * pro[3] < 0) {
        return true;
    }
    return false;
}

void compute_mask(const cv::Mat &mask, cv::Point2f rect_points[],
                  cv::Mat *submask) {
    *submask = cv::Mat::zeros(mask.size(), CV_8U);

    for (int i = 0; i < mask.rows; i++)
        for (int j = 0; j < mask.cols; j++) {
            if (mask.at<uchar>(i, j) > 0)
                if (isInROI(cv::Point(j, i), rect_points))
                    submask->at<uchar>(i, j) = 255;
        }
}

void MatchDescriptor::computeDescriptors(
    const cv::Mat &image, const cv::Mat &mask1,
    std::vector<std::vector<cv::Point> > &contours, cv::Mat *descriptors,
    int histSize, std::vector<cv::Mat> *submasks) {
    std::vector<cv::Mat> histograms_all;
    std::vector<std::vector<cv::Point> > localcontours;
    localcontours.assign(contours.begin(), contours.end());

    for (int icont = 0; icont < localcontours.size(); icont++) {
        cv::RotatedRect minRect =
            cv::minAreaRect(cv::Mat(localcontours[icont]));

        cv::Point2f rect_points[4];
        minRect.points(rect_points);
        cv::Mat submask;
        compute_mask(mask1, rect_points, &submask);

        float contmask = 0, contsubmask = 0;
        for (int i = 0; i < mask1.rows; i++)
            for (int j = 0; j < mask1.cols; j++)
                if (mask1.at<uchar>(i, j) > 0) contmask++;

        // compute the 1's in the image

        for (int i = 0; i < submask.rows; i++)
            for (int j = 0; j < submask.cols; j++)
                if (submask.at<uchar>(i, j) > 0) contsubmask++;
        printf("Cluster %d has %f area \n", icont, contsubmask / contmask);

        if ((contsubmask / contmask < 0.9) && (contsubmask / contmask > 0.1)) {
            //                     cv::imshow("submask", submask);
            //                     cv::waitKey(15);

            /// Establish the number of bins
            submasks->push_back(submask);

            // compute histograms
            float range[] = {0, 256};
            const float *histRange = {range};

            bool uniform = true;
            bool accumulate = false;

            cv::Mat b_hist, g_hist, r_hist;
            /// Separate the image in 3 places ( B, G and R )
            std::vector<cv::Mat> bgr_planes;
            cv::split(image, bgr_planes);

            /// Compute the histograms:
            cv::calcHist(&bgr_planes[0], 1, 0, submask, b_hist, 1, &histSize,
                         &histRange, uniform, accumulate);
            cv::calcHist(&bgr_planes[1], 1, 0, submask, g_hist, 1, &histSize,
                         &histRange, uniform, accumulate);
            cv::calcHist(&bgr_planes[2], 1, 0, submask, r_hist, 1, &histSize,
                         &histRange, uniform, accumulate);

            // normalize histograms to [0,1]
            cv::normalize(b_hist, b_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
            cv::normalize(g_hist, g_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
            cv::normalize(r_hist, r_hist, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

            cv::Mat histvect = cv::Mat::zeros(histSize * 3, 1, CV_32F);

            for (int k = 0; k < histSize * 3; k++) {
                if (k < histSize) histvect.at<float>(k) = r_hist.at<float>(k);
                if ((histSize <= k) && (k < histSize * 2))
                    histvect.at<float>(k) = g_hist.at<float>(k % histSize);
                if ((histSize * 2 <= k) && (k < histSize * 3))
                    histvect.at<float>(k) = b_hist.at<float>(k % histSize);
            }
            //       std::cout  << "hist all " << histvect << std::endl;
            double min, max;
            cv::minMaxIdx(histvect, &min, &max);
            if (max != 0) histograms_all.push_back(histvect);

        } else
            contours.erase(contours.begin() + icont);
    }

    *descriptors = cv::Mat::zeros(histograms_all.size(), histSize * 3, CV_32F);
    for (int i = 0; i < histograms_all.size(); i++)
        descriptors->row(i) = histograms_all[i].t();
}

bool MatchDescriptor::matchCallback(MatchCluster::Request &request,
                                    MatchCluster::Response &response) {
    // get the point cloud
    pcl::fromROSMsg(request.match.cluster.pointcloud, cluster_pc_);

    // remove NAN points from the cloud
    std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(cluster_pc_,cluster_pc_, indices);

    // get the list of possible objects
    int nobjects = request.match.object_ids.size();

    std::cout << "Number of objects in the list to be processed " << nobjects
              << std::endl;
    // get cluster reference frame id
    std::string cluster_frame_id = request.match.cluster.header.frame_id;
    ROS_INFO("cluster_frame_id: %s", cluster_frame_id.c_str());

    std::string error_msg;
    // obtain the transform from point cloud to image frame
    if (tf_.canTransform(camera_frame_id_, cluster_frame_id, ros::Time(0),
                         &error_msg)) {
        tf_.lookupTransform(camera_frame_id_, cluster_frame_id, ros::Time(0),
                            cloud_to_camera_);
    } else {
        ROS_WARN_THROTTLE(
            10.0,
            "The tf from  '%s' to bin '%s' does not seem to be available, "
            "will assume it as identity!",
            cluster_frame_id.c_str(), camera_frame_id_.c_str());
        ROS_WARN("Transform error: %s", error_msg.c_str());
        cloud_to_camera_.setIdentity();
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transf_pc(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(cluster_pc_, *transf_pc, cloud_to_camera_);

    cv::Mat image, mask;
    computeMask(transf_pc, &image, &mask);

    if (debug_) {
        cv::imshow("IMage", image);
        cvWaitKey(0);
    }

    // convert image to gray
    cv::Mat im_gray(image.size(), CV_8UC1);

    cvtColor(image, im_gray, CV_BGR2GRAY);
    // cv::blur( im_gray, im_gray, cv::Size(3,3) );

    cv::Mat blur, blurdepth;

    cv::GaussianBlur(im_gray, blur, cv::Size(7, 7), 15, 15);

    cv::Mat blurcolor;
    cv::cvtColor(blur, blurcolor, CV_GRAY2BGR);

    cv::Mat descriptors_MSER;
    std::vector<cv::KeyPoint> keypoints, keypointsdef;
    std::vector<std::vector<cv::Point> > keypointsMSER, keypointsMSERdef;

    std::vector<cv::Mat> submasks;

    cv::MSER mser(11, 150, 14400, .1, .8, 200, 1.3);

    mser(im_gray, keypointsMSER, mask);

    printf("Possible clusters detected = %d\n", (int)keypointsMSER.size());

    for (int i = 0; i < keypointsMSER.size(); i++) {
        cv::RotatedRect minRect = cv::minAreaRect(cv::Mat(keypointsMSER[i]));

        if (std::max(minRect.size.height, minRect.size.width) > 30)
            keypointsMSERdef.push_back(keypointsMSER[i]);
    }
    printf("Possible clusters filtered by size = %d\n",
           (int)keypointsMSERdef.size());

    int histSize = 8;

    computeDescriptors(image, mask, keypointsMSERdef, &descriptors_MSER,
                       histSize, &submasks);

    std::cout << "\n\n\nBLOBS DETECTED  " << keypointsMSERdef.size()
              << std::endl;
    matcher_ = new cv::BFMatcher(cv::NORM_L2);

    // std::cout <<  descriptors.rows << " descriptors computed " <<  std::endl;

    for (int i = 0; i < nobjects; i++) {
        // load the descriptors
        std::string obj_id = request.match.object_ids[i];
        std::cout << "\n\n----Processing object =  " << obj_id << std::endl;
        std::string filename(descriptors_dir_ + obj_id + "_MSER.yaml");
        std::cout << "Reading " << filename << std::endl;
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        cv::Mat train_descriptors_MSER;
        cv::Mat train_3dpoints;
        fs["Descriptors"] >> train_descriptors_MSER;
        fs["Points"] >> train_3dpoints;
        fs.release();

        //         if (train_3dpoints.rows != 1)
        //             train_3dpoints = train_3dpoints.t();

        //         std::cout << "Trained descriptor size " <<
        //         train_descriptors.cols <<  std::endl;
        //         std::cout << "Computed descriptor size " <<  descriptors.cols
        //         <<
        //         std::endl;

        std::vector<cv::DMatch> matches;
        std::vector<cv::DMatch> matchesdef;

        matcher_->match(descriptors_MSER, train_descriptors_MSER, matches);
        std::cout << "MSER matches " << matches.size() << std::endl;
        for (int j = 0; j < matches.size(); j++) {
            printf("---- Processing cluster %d/%d  ---\n", j + 1,
                   (int)matches.size());
            std::cout << "distance " << matches[j].distance << std::endl;
            if (matches[j].distance < descr_distance_) {
                matchesdef.push_back(matches[j]);
            }
        }

        // if MSER used, compute SIFT inside of subclusters
        std::string filename2(descriptors_dir_ + obj_id + "_" +
                              request.match.descriptor_type + ".yaml");

        std::cout << "Reading " << filename2 << std::endl;
        cv::FileStorage fs2(filename2, cv::FileStorage::READ);
        cv::Mat train_descriptors;
        // cv::Mat train_3dpoints;
        fs2["Descriptors"] >> train_descriptors;
        // fs["Points"] >> train_3dpoints;
        fs2.release();
        cv::Mat descriptors;
        keypoints.clear();
        std::cout << "ORB loaded " << std::endl;
        //
        double mindistMSER = 1000;
        int maxinliers = 0;
        int match = -1;
        std::vector<cv::DMatch> good_matches;

        for (int j = 0; j < matchesdef.size(); j++) {
            if (debug_) {
                cv::Mat img_matches;

                cv::Mat mask3C;
                cv::cvtColor(submasks[matchesdef[j].queryIdx], mask3C,
                             CV_GRAY2BGR);
                cv::addWeighted(image, 0.4, mask3C, 0.7, 0.0, img_matches);
                cv::imshow("MASK", img_matches);
                cv::waitKey(0);
            }

            if (mindistMSER > matchesdef[j].distance)
                mindistMSER = matchesdef[j].distance;
            std::vector<cv::KeyPoint> keypointslocal;
            if (request.match.descriptor_type == "SIFT") {
                cv::SIFT sift;
                sift(image, submasks[matchesdef[j].queryIdx], keypointslocal,
                     descriptors);
                matcher_ = new cv::FlannBasedMatcher();
            }
            if (request.match.descriptor_type == "ORB") {
                cv::ORB orb(100, 1.2, 8, 15, 0, 2, cv::ORB::HARRIS_SCORE, 15);
                orb(image, submasks[matchesdef[j].queryIdx], keypointslocal,
                    descriptors);
            }

            std::cout << request.match.descriptor_type
                      << " descriptors computed " << descriptors.rows
                      << std::endl;
            std::vector<std::vector<cv::DMatch> > matchesknn;
            std::vector<cv::DMatch> matches_des;
            matcher_->match(descriptors, train_descriptors, matches_des);
            // matcher_->knnMatch(descriptors, train_descriptors,
            // matchesknn,5,cv::Mat(),false);
            int inliers = 0;

            double max_dist = 0;
            double min_dist = 200;
            // normal matches
            for (int i = 0; i < descriptors.rows; i++) {
                double dist = matches_des[i].distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }
            printf("-- Max dist : %f \n", max_dist);
            printf("-- Min dist : %f \n", min_dist);

            std::vector<cv::DMatch> good_matches_local;

            for (int i = 0; i < descriptors.rows; i++) {
                if (matches_des[i].distance <=
                    std::max(min_dist + (max_dist - min_dist) / 2., 0.02)) {
                    good_matches_local.push_back(matches_des[i]);
                    inliers++;
                }
            }
            printf("Inliers detected =  %d\n", inliers);
            if (maxinliers < inliers) maxinliers = inliers;
            if ((inliers > min_matches_) &&
                (inliers ==
                 maxinliers))  // && (mindistMSER == matchesdef[j].distance))
            {
                match = j;
                good_matches.clear();
                good_matches.assign(good_matches_local.begin(),
                                    good_matches_local.end());
                keypoints.clear();
                keypoints.assign(keypointslocal.begin(), keypointslocal.end());
                std::cout << obj_id << "possible  found" << std::endl;

                /******************************************/
                // FILL THE RESPONSE HERE
                // IT COULD BE THE SUBCLUSTER GIVEN BY THE SUBMASK OF THE
                // MATCHED OBJECT
                // cluster <--  submasks[matchesdef[match]]
                /******************************************/
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcl;
                subcl =
                    subcluster(submasks[matchesdef[match].queryIdx], transf_pc);
                std::string cluster_id = obj_id;
            }
        }
        //    std::cout << obj_id << "  found" << std::endl;

        if (debug_) {
            cv::Mat img_matches;
            std::vector<cv::KeyPoint> keypoints_local;
            for (int i = 0; i < good_matches.size(); i++) {
                keypoints_local.push_back(keypoints[good_matches[i].queryIdx]);
            }
            cv::Mat mask3C;
            cv::cvtColor(submasks[matchesdef[match].queryIdx], mask3C,
                         CV_GRAY2BGR);
            cv::addWeighted(image, 0.4, mask3C, 0.7, 0.0, img_matches);
            cv::drawKeypoints(img_matches, keypoints_local, img_matches);
            cv::imshow("matches", img_matches);
            cvWaitKey(0);
        }
    }
}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "match_object");

    ros::NodeHandle nh;
    pr2_pick_perception::MatchDescriptor matcher;

    if (!matcher.initialize()) {
        ROS_FATAL("Object matcher initialization failed. Shutting down node.");
        return 1;
    }

    ros::ServiceServer server(nh.advertiseService(
        "match_cluster", &pr2_pick_perception::MatchDescriptor::matchCallback,
        &matcher));

    ros::spin();

    return 0;
}
