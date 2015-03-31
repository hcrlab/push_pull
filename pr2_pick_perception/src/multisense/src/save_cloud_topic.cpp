#include <ros/ros.h>
#include <pcl_ros/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <pcl/ros/conversions.h>
#include <ros/publisher.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

std::string cloud_filename = "hansi.pcd";
bool extract = false;


void pointCloudSaver(const sensor_msgs::PointCloud2ConstPtr &pcl_msg)
{    
    if(extract)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pcl_msg,*cloud);        
        
        //extract region
        ROS_INFO("extracting region from point cloud\n");
        pcl::PassThrough<pcl::PointXYZ> pass_x, pass_y, pass_z;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(0.1, 10);
        pass_x.setKeepOrganized(false); //should remove NaN and Inf values
        pass_x.setInputCloud(cloud);
        pass_x.filter(*cloud);
        
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-10, 10);
        pass_y.setKeepOrganized(false); //should remove NaN and Inf values
        pass_y.setInputCloud(cloud);
        pass_y.filter(*cloud);
        
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-5, 5);
        pass_z.setKeepOrganized(false); //should remove NaN and Inf values
        pass_z.setInputCloud(cloud);
        pass_z.filter(*cloud);   
        
        ROS_INFO("saving cloud to %s\n", cloud_filename.c_str());
        pcl::io::savePCDFile(cloud_filename, *cloud);
        ROS_INFO("saved cloud %s\n", cloud_filename.c_str());
    }
    else
    {
        ROS_INFO("saving cloud to %s\n", cloud_filename.c_str());
        pcl::io::savePCDFile(cloud_filename, *pcl_msg);
        ROS_INFO("saved cloud %s\n", cloud_filename.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_cloud_topic");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");    
    
    //update frequency in seconds
    double freq = 0.5;
    local_nh.getParam("freq", freq);
    local_nh.getParam("extract", extract);
    ROS_INFO("Collecting snapshots every %.2f seconds\n", freq);
    ROS_INFO("Extracting volume: %d\n", extract);
    
    //get pcd filename
    local_nh.getParam("output", cloud_filename);
    
    //subscribe to topic
    std::string pcl_topic = nh.resolveName("cloud");
    ros::Subscriber pcl_sub = nh.subscribe(pcl_topic, 1, pointCloudSaver);

    
    ros::Rate r(1.0/freq); //convert seconds to Hz
    while(nh.ok())
    {        
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
