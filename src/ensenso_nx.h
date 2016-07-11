#ifndef ENSENSO_NX_H
#define ENSENSO_NX_H

//Ensenso Nx Lib
#include "nxLib.h"

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//std
#include <iostream>

namespace EnsensoNx
{

struct DeviceParams
{
    std::string ip_address_;    
    std::string model_name_; 
    unsigned int sn_; 
    
    void print() const
    {
        std::cout << "\tIP address: \t" << ip_address_ << std::endl;
        std::cout << "\tModel: \t" << model_name_ << std::endl;
        std::cout << "\tSN: \t" << sn_ << std::endl;    
    }
};

//device configuration struct
struct CaptureParams
{
    bool auto_exposure_;
    double exposure_time_; //in useconds
    
    void print() const
    {
        std::cout << "\tauto exposure [t/f]: \t" << auto_exposure_ << std::endl;
        std::cout << "\texposure [us]: \t" << exposure_time_ << std::endl;
    }
};


class Device
{
    protected: 
        
        DeviceParams device_params_; //params related to device operations      
        CaptureParams capture_params_; //params related to point cloud acquisition
        
        NxLibItem nx_lib_root_; // Reference to the API tree root 
        NxLibItem camera_; //Reference to the nxlib camera device

    public: 
        /** \brief Constructor
         * Constructor
        **/
        Device(const DeviceParams & _params); 

        /** \brief Destructor
         * Destructor
        **/
        ~Device(); 
        
        /** \brief Set configuration for point cloud acquisition
         * Set configuration for point cloud acquisition
         * \param _params: capture parameters 
        **/
        void configureCapture(const CaptureParams & _params);
        
        /** \brief Get a point cloud from the device
         * Get a point cloud from the device
        **/
        void capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud);
        
}; //end class

} // end namespace

#endif

