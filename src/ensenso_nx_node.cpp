#include "ensenso_nx_node.h"

EnsensoNxNode::EnsensoNxNode():
    nh_(ros::this_node::getName()), 
    image_tp_(nh_)
{    
    int param_int; 
    
    //init the point cloud publisher
    cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("ensenso_cloud", 1);
    
    //init the image publisher
    image_publisher_ = image_tp_.advertise("ensenso_depth_image", 1);
    
    //init server
    cloud_server_ = nh_.advertiseService("ensenso_server", &EnsensoNxNode::pointCloudServiceCallback, this);
    
    //create pointer to ensenso device
    camera_ = new EnsensoNx::Device(); 

    //configure node according yaml params
    nh_.getParam("run_mode", param_int); this->run_mode_ = (RunMode)param_int; 
    //std::cout << __LINE__ << ": param_int: " << param_int << "; run_mode_: " << run_mode_ << std::endl; 
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->frame_name_);
    nh_.getParam("auto_exposure", this->capture_params_.auto_exposure_);
    nh_.getParam("exposure_time", param_int); this->capture_params_.exposure_time_ = (unsigned int)param_int;
    nh_.getParam("dense_cloud", param_int); this->capture_params_.dense_cloud_ = (bool)param_int;
    if ( run_mode_ == PUBLISHER ) 
    {
        camera_->configureCapture(this->capture_params_);
    }
    
    //print configs 
    std::cout << "ROS EnsensoNxNode Settings: " << std::endl; 
    std::cout << "\trun mode: \t" << run_mode_ << std::endl;
    std::cout << "\tframe name: \t" << frame_name_ << std::endl;
    if ( run_mode_ == PUBLISHER ) //in SERVER, rate is not applicable, and other capture params are set at the request message
    {
        std::cout << "\trate [hz]: \t" << rate_  << std::endl;
        std::cout << "\tauto_exposure [hz]: \t" << capture_params_.auto_exposure_ << std::endl;
        if ( !capture_params_.auto_exposure_ )
        {
            std::cout << "\texposure [ms]: \t" << capture_params_.exposure_time_ << std::endl;
        }
        std::cout << "\tdense_cloud: [t/f] \t" << capture_params_.dense_cloud_ << std::endl;
    }
}

EnsensoNxNode::~EnsensoNxNode()
{
    //destroy pointer to ensenso device
    delete camera_;
}

RunMode EnsensoNxNode::runMode() const
{
    return run_mode_;    
}

double EnsensoNxNode::rate() const
{
    return rate_;
}

void EnsensoNxNode::publish()
{
    //Get a single capture from camera and publish the point cloud
    if ( camera_->capture(cloud_, image_.image) == 1 )
    {
        //get time
        ros::Time ts = ros::Time::now();
        
        //publish the cloud
        cloud_.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
        cloud_.header.frame_id = frame_name_; 
        cloud_publisher_.publish(cloud_);
        
        //publish the depth image
        image_.header.seq ++;
        image_.header.stamp = ts;
        image_.header.frame_id = frame_name_; 
        image_.encoding = sensor_msgs::image_encodings::MONO16;
//         image_.encoding = sensor_msgs::image_encodings::MONO8;
        image_publisher_.publish(image_.toImageMsg());        
    }
    else
    {
        std::cout << "EnsensoNxNode::publish(): Error with point cloud capture" << std::endl;
    }
        
}
        
bool EnsensoNxNode::pointCloudServiceCallback(sensor_msgs::SnapshotCloud::Request  & _request, 
                                              sensor_msgs::SnapshotCloud::Response & _reply)
{
    //configure capture according request
    if (_request.exposure == 0)
    {
        capture_params_.auto_exposure_ = true; 
    }
    else
    {
        capture_params_.auto_exposure_ = false;
        capture_params_.exposure_time_ = _request.exposure; 
    }
    capture_params_.dense_cloud_ = _request.dense_cloud;
    camera_->configureCapture(capture_params_);
    
    //Get a single capture from camera and set the reply
    if ( camera_->capture(cloud_) == 1 )
    {
        //get time
        ros::Time ts = ros::Time::now();        
        cloud_.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
        cloud_.header.frame_id = frame_name_;        
        pcl::toROSMsg(cloud_, _reply.cloud); //see pcl-ros conversions at http://wiki.ros.org/action/fullsearch/pcl/Overview
    }
    else
    {
        std::cout << "EnsensoNxNode::pointCloudServiceCallback(): Error while capturing point cloud" << std::endl;
    }

    //return
    return true; 
}
