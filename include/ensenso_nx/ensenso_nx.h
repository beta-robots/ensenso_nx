#ifndef ___ENSENSO_NX___ENSENSO_NX_H___
#define ___ENSENSO_NX___ENSENSO_NX_H___

#include <iostream>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <nxLib.h>


namespace ensenso_nx
{

struct DeviceParams
{
	std::string serial_num;

	void print() const
	{
		std::cout << "\tSN: \t" << serial_num << std::endl;
	}
};

struct CaptureParams
{
	bool auto_exposure;
	unsigned int exposure_time; //in milliseconds TODO: check if uint is enough, or needs double
	int flex_view = 0;
	bool dense_cloud; //Device::capture() returns a dense (ordered) point cloud if set to true

	void print() const
	{
		std::cout << "\tauto exposure [t/f]: \t" << auto_exposure << std::endl;
		std::cout << "\texposure [ms]: \t" << exposure_time << std::endl;
		std::cout << "\tdense cloud [t/f]: \t" << dense_cloud << std::endl;
	}
};


class Device
{
protected:

	DeviceParams device_params__;
	CaptureParams capture_params__;

	NxLibItem nx_lib_root__;
	NxLibItem camera__;

public:
	/** \brief Constructor with serial number
	 * Constructor with serial number
	**/
	Device(const std::string & __serial_num);

	/** \brief Destructor
	 * Destructor
	**/
	~Device();

	/** \brief Set configuration for point cloud capture
	 * Set configuration for point cloud capture
	 * \param _params: capture parameters
	**/
	void configureCapture(const CaptureParams & __params);

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
	int capture(pcl::PointCloud<pcl::PointXYZ> & __p_cloud);
	int capture(pcl::PointCloud<pcl::PointXYZI> & __p_cloud);
	int capture(pcl::PointCloud<pcl::PointXYZRGB> & __p_cloud);


protected:
	/** \brief Hardware set configuration
	 * Hardware set configuration
	 * \param _params: capture parameters
	**/
	void configureCapture();
	bool flexview_enabled__ = false ;


};

}

#endif
