
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <trooper_adaptive_perception_msgs/SelectObject.h>
#include <cv_bridge/cv_bridge.h>

std::string src_window = "gray";
cv::Point mouse_point1_, mouse_point2_;
bool click1_,click2_;

void callbackmouse(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN)// && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
	if (!click1_ && !click2_){
	  mouse_point1_ = cv::Point(x, y);
	  click1_ = true;
	}else if(click1_ && !click2_){
	   mouse_point2_ = cv::Point(x, y);
	   click2_ = true;  
	}
        //drag = 1;
	
	//std::cout << "the click potin is = " << x << "," << y<< std::endl;
    }

    if (event == CV_EVENT_MOUSEMOVE)// && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        /*cv::Mat img1 = frame.clone();
        point2 = cv::Point(x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        cv::imshow(src_window, img1);
	std::cout << "the dragged point is = " << x << "," << y<< std::endl;
	cv::waitKey(4);*/
    }

    if (event == CV_EVENT_LBUTTONUP)// && drag && !select_flag)
    {
        /*cv::Mat img2 = frame.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
        cv::imshow(src_window, img2);
	cv::waitKey(4);
        callback_im = true;*/
    }
    if (event ==CV_EVENT_RBUTTONDOWN){
      /*callback_im =false;
      select_flag = false;  */
      
    }
}







int main(int argc, char** argv) {
  
  
  ros::init (argc, argv, "process_image");    
  
  ros::NodeHandle nh;
  
  ros::ServiceClient client_images = nh.serviceClient<trooper_adaptive_perception_msgs::SelectObject>("/ap/select_obect");
  
  trooper_adaptive_perception_msgs::SelectObject srv;
  std_msgs::String trigger;
  
  srv.request.input = trigger;
  client_images.call(srv);
  
  
  cv::Mat H,gray,mask;
  cv_bridge::CvImageConstPtr cvImage;
  cvImage = cv_bridge::toCvCopy(srv.response.grayImage,sensor_msgs::image_encodings::MONO8);
  cvImage->image.copyTo(gray);
  cvImage = cv_bridge::toCvCopy(srv.response.mask,sensor_msgs::image_encodings::MONO8);
  cvImage->image.copyTo(mask);
  cvImage = cv_bridge::toCvCopy(srv.response.HImage,sensor_msgs::image_encodings::MONO8);
  cvImage->image.copyTo(H);
  
  
  cv::namedWindow(src_window);
  cv::setMouseCallback(src_window,callbackmouse,0);
  cv::imshow(src_window,gray);
  cv::waitKey(0);
  
   
  cv::Rect rect1,rect2;
  
  if(click1_)
    cv::floodFill(H,mask,mouse_point1_,cv::Scalar(255),&rect1,cv::Scalar(2),cv::Scalar(2));
  if(click2_)
    cv::floodFill(H,mask,mouse_point2_,cv::Scalar(255),&rect2,cv::Scalar(2),cv::Scalar(2));

  cv::imshow(src_window,H);            
  cv::waitKey();
  cv::destroyWindow(src_window);
  

  return 0;
  
}