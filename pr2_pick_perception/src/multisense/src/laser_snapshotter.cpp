#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl/range_image/range_image.h>
#include <pcl/ros/conversions.h>
#include <ros/publisher.h>


using namespace laser_assembler;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasercloud_to_file");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");    
    
    //update frequency in seconds
    double freq = 10;
    local_nh.getParam("freq", freq);
    ROS_INFO("Collecting snapshots every %.2f seconds\n", freq);
    
    //advertise assembled point cloud
    std::string pub_topic = nh.resolveName("assembled_cloud");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
    
    //Subscribe to assembler service
    std::string assembler_service = "assemble_scans2";
    local_nh.getParam("assembler_service", assembler_service);
    ros::service::waitForService(assembler_service);
    ros::ServiceClient client = nh.serviceClient<AssembleScans2>(assembler_service);

    ros::Rate r(1.0/freq); //convert seconds to Hz
    while(nh.ok())
    {
        AssembleScans2 srv;
//         srv.request.begin = ros::Time::now()-ros::Duration(freq);
        srv.request.begin = ros::Time(0);
        srv.request.end   = ros::Time::now();
        if (client.call(srv))
            ROS_DEBUG("Got cloud with %lu points\n", srv.response.cloud.data.size());
        else
            ROS_ERROR("Service call failed\n");
        
        //create point cloud from msg
        //sensor_msgs::PointCloud2 cloud;
        //sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud);
        
        pcl_pub.publish(srv.response.cloud);
        
        r.sleep();
    }
    
    return 0;
}
