#ifndef ___ENSENSO_NX___ENSENSO_NX_NODE_H___
#define ___ENSENSO_NX___ENSENSO_NX_NODE_H___

#include <iostream>
#include <mutex>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_srvs/SetBool.h>
#include <sensor_msgs/SnapshotCloud.h> // forked at https://github.com/beta-robots/common_msgs

#include <dynamic_reconfigure/server.h>
#include <ensenso_nx/EnsensoNxParamsConfig.h>
#include <ensenso_nx/ensenso_nx.h>


namespace ensenso_nx
{

// Runnning modes
static const int SERVER_MODE = 1;
static const int PUBLISHER_MODE = 2;

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
public:
	EnsensoNxNode();
	~EnsensoNxNode();

protected:
	std::shared_ptr<Device> camera__;
	CaptureParams capture_params__;
	bool grabCloud();
	std::mutex grab_locker__;

	ros::NodeHandle nh__;
	dynamic_reconfigure::Server<EnsensoNxParamsConfig> configure_server__;
	void reconfigureCallback(EnsensoNxParamsConfig& __config, uint32_t __level);

	bool is_camera_enabled__;
	ros::ServiceServer set_camera_service__;
	bool setCameraEnable(std_srvs::SetBoolRequest& __req, std_srvs::SetBoolResponse& __res);

	ros::Timer publisher_clock__;
	void publisherCallBack(const ros::TimerEvent& __e);

	ros::ServiceServer cloud_server__;
	bool pointCloudServiceCallback(sensor_msgs::SnapshotCloud::Request  & __req, sensor_msgs::SnapshotCloud::Response & __res);

	ros::Publisher cloud_publisher__;
	pcl::PointCloud<pcl::PointXYZ> cloud__;

	bool is_params_loaded__;
	int run_mode__;
	double rate__;
	std::string frame_name__;
	std::string serial_number__;
};

}
#endif
