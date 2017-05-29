#ifndef ENSENSO_NX_H
#define ENSENSO_NX_H

//Ensenso Nx Lib
#include "nxLib.h"

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//std
#include <iostream>
#include <cmath>

namespace EnsensoNx
{

//device configuration struct
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

//capture configuration struct
struct CaptureParams
{
    bool auto_exposure_;
    unsigned int exposure_time_; //in milliseconds TODO: check if uint is enough, or needs double
    bool dense_cloud_; //Device::capture() returns a dense (ordered) point cloud if set to true

    void print() const
    {
        std::cout << "\tauto exposure [t/f]: \t" << auto_exposure_ << std::endl;
        std::cout << "\texposure [ms]: \t" << exposure_time_ << std::endl;
        std::cout << "\tdense cloud [t/f]: \t" << dense_cloud_ << std::endl;
    }
};


class Device
{
    protected:

        DeviceParams device_params_; //params related to device operations
        CaptureParams capture_params_; //params related to point cloud acquisition

        NxLibItem nx_lib_root_; // Reference to the API tree root
        NxLibItem camera_; //Reference to the nxlib camera device

        std::vector<float> raw_points_; //raw xyz points from camera

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
//         void configureExposure(unsigned int _exposure);

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
