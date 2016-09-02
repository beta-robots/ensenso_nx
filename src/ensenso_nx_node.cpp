#include "ensenso_nx_node.h"

EnsensoNxNode::EnsensoNxNode():
    nh_(ros::this_node::getName())
{    
    int param_int; 
    
    //init the point cloud publisher
    cloud_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("ensenso_cloud", 1);
    
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
    if ( run_mode_ == PUBLISHER ) 
    {
        camera_->configureCapture(this->capture_params_);
    }
    
//     run_mode_ = SERVER; 
//     rate_ = 10; 
//     frame_name_ = "single_ensenso_n35_body"; 
    
    //print configs 
    std::cout << "ROS EnsensoNxNode Settings: " << std::endl; 
    std::cout << "\trun mode: \t" << run_mode_ << std::endl;
    std::cout << "\tframe name: \t" << frame_name_ << std::endl;
    if ( run_mode_ == PUBLISHER )
    {
        std::cout << "\trate [hz]: \t" << rate_  << std::endl;
        std::cout << "\tauto_exposure [hz]: \t" << capture_params_.auto_exposure_ << std::endl;
        if ( !capture_params_.auto_exposure_ )
        {
            std::cout << "\texposure [ms]: \t" << capture_params_.exposure_time_ << std::endl;
        }
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
    if ( camera_->capture(cloud_) == 1 )
    {
        ros::Time ts = ros::Time::now();
        cloud_.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
        cloud_.header.frame_id = frame_name_; 
        cloud_publisher_.publish(cloud_);
    }
    else
    {
        std::cout << "EnsensoNxNode::publish(): Error with point cloud capture" << std::endl;
    }
        
}
        
bool EnsensoNxNode::pointCloudServiceCallback(ensenso_nx::PointCloudAsService::Request  & _request, 
                                              ensenso_nx::PointCloudAsService::Response & _reply)
{
    //set the exposure 
    camera_->configureExposure((unsigned int)_request.exposure);
    
    //call capture and publish point cloud
    this->publish();
    
    //set the reply
    _reply.size = cloud_.size(); 
    
    //return
    return true; 
}
