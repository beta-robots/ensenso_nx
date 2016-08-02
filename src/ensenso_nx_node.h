#ifndef ensenso_nx_node_H
#define ensenso_nx_node_H

//std
#include <iostream>

//this package
#include "ensenso_nx.h"

//std ros dependencies
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h> //PCL-ROS interoperability
#include <pcl_conversions/pcl_conversions.h> //conversions from/to PCL/ROS
#include <std_msgs/Empty.h> //snapshot request

//custom ROS dependencies
#include "ensenso_nx/PointCloudAsService.h" //custom "capture" service
//#include <ensenso_nx/ensenso_nx_paramsConfig.h> //ROS dynamic configure

//enum run mode
enum RunMode {SERVER=0,PUBLISHER};

/** \brief Ensenso NX ROS wrapper
 * 
 * Ensenso NX ROS wrapper
 * 
 * Two running modes:
 *    * Snapshot upon request
 *    * Continuous point cloud publisher (not yet implemented) 
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
        
        //A pcl point cloud, used to get data and publish it
        pcl::PointCloud<pcl::PointXYZ> cloud_; 
        
        //node parameters
        double rate_; //loop rate
        std::string frame_name_; //name of the frame of references with respect cloud are published
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher
                
        //device parameters
        EnsensoNx::DeviceParams device_params_;
        
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
        bool pointCloudServiceCallback(ensenso_nx::PointCloudAsService::Request  & _request, 
                                      ensenso_nx::PointCloudAsService::Response & _reply);
                
};
#endif
