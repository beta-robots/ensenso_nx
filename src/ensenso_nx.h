#ifndef ENSENSO_NX_H
#define ENSENSO_NX_H

//Ensenso Nx Lib
#include "nxLib.h"

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//std
#include <iostream>
#include <math.h>

namespace EnsensoNx
{

struct DeviceParams
{
//     std::string ip_address_;    
//     std::string model_name_; 
    std::string serial_num_; 
    
    void print() const
    {
//         std::cout << "\tIP address: \t" << ip_address_ << std::endl;
//         std::cout << "\tModel: \t" << model_name_ << std::endl;
        std::cout << "\tSN: \t" << serial_num_ << std::endl;    
    }
};

//device configuration struct
struct CaptureParams
{
    bool auto_exposure_;
    double exposure_time_; //in microseconds
    
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
        Device(); 

        /** \brief Destructor
         * Destructor
        **/
        ~Device(); 
        
        /** \brief Set configuration for point cloud capture
         * Set configuration for point cloud capture
         * \param _params: capture parameters 
        **/
        void configureCapture(const CaptureParams & _params);
        
        /** \brief Set exposure in microseconds
         * Set exposure in microseconds
         * \param _exposure: exposure in milliseconds, a value of 0 indicates autoexposure
        **/
        void configureExposure(unsigned int _exposure);
        
        /** \brief Get a point cloud from the device
         * Get a point cloud from the device
         * \param _p_cloud: a point cloud where the capture data will be placed
         * \return 1 if ok, -1 if not. TODO
        **/
        int capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud);
        
    protected:
        /** \brief Hardware set configuration
         * Hardware set configuration
         * \param _params: capture parameters 
        **/        
        void configureCapture(); 
        
}; //end class

} // end namespace

#endif

