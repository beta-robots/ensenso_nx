#ifndef ensenso_nx_node_H
#define ensenso_nx_node_H

//std
#include <iostream>
#include <sstream>

//this package
#include "ensenso_nx.h"

//std ros dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
//#include <std_msgs/Empty.h> //snapshot request
#include <sensor_msgs/SnapshotCloud.h> //forked at https://github.com/beta-robots/common_msgs

//this package dependencies
#include <ensenso_nx/ensenso_nx_paramsConfig.h> //ROS dynamic configure

//enum run mode
enum RunMode {SERVER=0,PUBLISHER};

/** \brief Ensenso NX ROS wrapper for point cloud capture
 *
 * Ensenso NX ROS wrapper for point cloud capture
 *
 * Two running modes:
 *    * SERVER: Snapshot upon request
 *    * PUBLISHER: Continuous point cloud publication
 *
 * In both cases the point cloud is published thorugh the same topic
 *
 **/
class EnsensoNxNode
{
    protected:
        //Device object with HW API
        EnsensoNx::Device *camera_;

        //ros node handle
        ros::NodeHandle nh_;

        //capture server
        ros::ServiceServer cloud_server_;

        //Publisher. Point Clouds are published through this topic
        ros::Publisher cloud_publisher_;

        //published point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud_;

        //node configuration parameters
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher
        double rate_; //loop rate
        std::string frame_name_; //name of the frame of references with respect cloud are published

        //device parameters TODO: think if it is necessary, ... it seems not!
        //EnsensoNx::DeviceParams device_params_;

        //capture parameters
        EnsensoNx::CaptureParams capture_params_;

    public:
        //constructor
        EnsensoNxNode();

        //destructor
        ~EnsensoNxNode();

        //returns run_mode_
        RunMode runMode() const;

        //returns rate_ value
        double rate() const;

        //Call to device snapshot acquisition and publish the point cloud
        void publish();

    protected:
        //Service callback implementing the point cloud capture
        bool pointCloudServiceCallback(sensor_msgs::SnapshotCloud::Request  & _request,
                                       sensor_msgs::SnapshotCloud::Response & _reply);

};
#endif
