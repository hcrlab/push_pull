#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

//OpenCV

#include <opencv2/highgui/highgui.hpp>

#include <stdlib.h>

int readStereoCalib(std::string calibFile,cv::Mat &K, cv::Mat &dist, cv::Mat &R, cv::Mat &t){

    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
       
    fs["cameraMatrix"] >> K;
    
    fs["distCoeffs"] >> dist;
    
    fs["RotationMatrix"] >> R;
    fs["Translation"] >> t;
    
    fs.release();
    return 0;

}



int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "realsense_camera_info");
 
    ros::NodeHandle nh;
 
    ros::NodeHandle nh_local("~");
 
    double width, height;
    nh_local.param("image_width", width, 640.0);
    nh_local.param("image_height", height, 480.0);
    
    std::string calibFile;
    
    nh_local.param("calibration_file",calibFile,std::string("realsense.yaml"));
    
    cv::Mat K(3,3,CV_64F),R(3,3,CV_64F),dist(3,1,CV_64F),t(3,1,CV_64F);    

    //read calibration
    std::cout << "Reading yaml file ... " << std::endl;
    readStereoCalib(calibFile.c_str(),K,dist,R,t);
      
    std::string camera_info_topic = nh.resolveName("/camera_info");
    ros::Publisher  camera_info_node = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic,1);

    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = "realsense_frame";
    
    for (int i=0; i < 9 ; i++)
    {
        camera_info.K[i] = K.at<double>(i);
        camera_info.R[i] = R.at<double>(i);
    }
    
    cv::Mat P(3,4,CV_64F);
    
    K.copyTo(P(cv::Rect(0,0,3,3)));
    //P.at<double>(0,3) = -camera_info.K[0] * t.at<double>(0);    
    P.at<double>(0,3) = t.at<double>(0);    
    P.at<double>(1,3) = t.at<double>(1);    
    P.at<double>(2,3) = t.at<double>(2);    
                 
    for (int i=0; i < 12;i ++)
        camera_info.P[i] = P.at<double>(i); 
    
    camera_info.width = width;
    camera_info.height = height;
    camera_info.header.stamp = ros::Time::now();
    camera_info.distortion_model = std::string("plumb_bob");
    
    camera_info.D.push_back(dist.at<double>(0));
    camera_info.D.push_back(dist.at<double>(1));
    camera_info.D.push_back(dist.at<double>(2));
    camera_info.D.push_back(0.0);
    camera_info.D.push_back(0.0);
    
    ros::Rate loop_rate(1);
    while(1)
    {
        camera_info_node.publish(camera_info);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}