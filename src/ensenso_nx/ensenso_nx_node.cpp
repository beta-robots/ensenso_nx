#include <ensenso_nx/ensenso_nx_node.h>

namespace ensenso_nx
{

EnsensoNxNode::EnsensoNxNode():
	nh__()
{
	is_params_loaded__ = false;

	if(!nh__.getParam("serial_number", serial_number__))
		ROS_ERROR_NAMED("EnensoNxNode", "Serial number for Ensenso camera has not been set, the camera won't be able to open");

	run_mode__ = SERVER_MODE;
	if( !nh__.getParam("run_mode", run_mode__) )
		ROS_WARN_NAMED("EnensoNxNode", "Run mode param is not set, the camera will start in SERVER mode by default");

	rate__ = 1.0;
	if( !nh__.getParam("rate", rate__) )
		ROS_WARN_NAMED("EnensoNxNode", "Rate param is not set, it will be set to 1Hz");

	frame_name__ = "single_ensenso_n35_body";
	if( !nh__.getParam("frame_name", frame_name__) )
		ROS_WARN_NAMED("EnensoNxNode", "frame_name is not set, it will be set to single_ensenso_n35_body");

	capture_params__.auto_exposure = false;
	if( !nh__.getParam("auto_exposure", capture_params__.auto_exposure) )
		ROS_WARN_NAMED("EnensoNxNode", "auto_exposure is not set, it will be set to false");

	capture_params__.flex_view = 0;
	if( !nh__.getParam("flexview", capture_params__.flex_view) )
		ROS_WARN_NAMED("EnensoNxNode", "flexview is not set, it will be set to false");

	int exposure_time = 0;
	if( !nh__.getParam("exposure_time", exposure_time) )
		ROS_WARN_NAMED("EnensoNxNode", "exposure_time is not set, it will be set to 0 (zero)");
	capture_params__.exposure_time = static_cast<unsigned int>(exposure_time);

	capture_params__.dense_cloud = false;
	if( !nh__.getParam("dense_cloud", capture_params__.dense_cloud) )
		ROS_WARN_NAMED("EnensoNxNode", "dense_cloud is not set, it will be set to 0 (false)");

	//calibration params
	he_cal_params__.decode_data = true;
	if( !nh__.getParam("cal_decode_data", he_cal_params__.decode_data) )
		ROS_WARN_NAMED("EnensoNxNode", "Decode data param for Hands eye calibration not found. it will be set to true;");

	he_cal_params__.grid_spacing = 0.0;
	if( !nh__.getParam("cal_grid_spacing", he_cal_params__.grid_spacing) )
		ROS_WARN_NAMED("EnensoNxNode", "Grid spacing for Hands eye not found. If decode data iss set to false the hands eye calibration will not work!");

	is_params_loaded__ = true;

	set_camera_service__ = nh__.advertiseService("set_camera", &EnsensoNxNode::setCameraEnable, this);

	cloud_publisher__ = nh__.advertise<pcl::PointCloud<pcl::PointXYZ> >("ensenso_cloud", 1);

	cloud_server__ = nh__.advertiseService("ensenso_server", &EnsensoNxNode::pointCloudServiceCallback, this);

	publisher_clock__ = nh__.createTimer( ros::Rate(rate__), &EnsensoNxNode::publisherCallBack, this, false, false);

	configure_server__.setCallback( boost::bind(&EnsensoNxNode::reconfigureCallback, this, _1, _2) );

	//ROS_WARN_STREAM("NODE NAME: "<<ros::this_node::getName());
	snapshot_action__.reset(new actionlib::SimpleActionServer<sensor_msgs::AdvancedSnapshotCloudAction>(ros::NodeHandle(), ros::this_node::getName() + "/advanced_snapshot/", boost::bind(&EnsensoNxNode::advancedSnapshotCallback, this, _1), false));
	snapshot_action__->start();

	handseye_calibration_action__.reset(new actionlib::SimpleActionServer<ensenso_nx::HECalibrationAction>(ros::NodeHandle(), ros::this_node::getName() + "/handseye_calibration/", boost::bind(&EnsensoNxNode::HandsEyeCalibrationCallback, this, _1), false));
	handseye_calibration_action__->start();

	std::cout << "ROS EnsensoNxNode Settings: " << std::endl;
	std::cout << "\trun mode: \t" << run_mode__<< std::endl;
	std::cout << "\tframe name: \t" << frame_name__ << std::endl;
	if ( run_mode__ == PUBLISHER_MODE )
	{
		std::cout << "\trate [hz]: \t" << rate__  << std::endl;
		std::cout << "\tauto_exposure [t/f]: \t" << capture_params__.auto_exposure << std::endl;
		if ( !capture_params__.auto_exposure )
		{
			std::cout << "\texposure [ms]: \t" << capture_params__.exposure_time << std::endl;
		}
		std::cout << "\tdense_cloud: [t/f] \t" << capture_params__.dense_cloud << std::endl;
	}

	// autostart
	is_camera_enabled__ = false;
	std_srvs::SetBool enable_camera;
	enable_camera.request.data = true;
	ROS_WARN("Enabling camera");
	setCameraEnable(enable_camera.request, enable_camera.response);
	ROS_WARN("End camera constructor");
}

EnsensoNxNode::~EnsensoNxNode()
{
	if(is_camera_enabled__)
	{
		std_srvs::SetBool close_camera;
		close_camera.request.data = false;
		setCameraEnable(close_camera.request, close_camera.response);
	}
}

void EnsensoNxNode::reconfigureCallback(EnsensoNxParamsConfig& __config, uint32_t __level)
{
	switch(__level)
	{
	case 0:
	{
		std_srvs::SetBool enable_camera;
		if( __config.enable_camera )
			enable_camera.request.data = true;
		else
			enable_camera.request.data = false;
		setCameraEnable(enable_camera.request, enable_camera.response);
		break;
	}
	case 1:
	{
		if( is_camera_enabled__ )
		{
			if( run_mode__ == PUBLISHER_MODE )
			{
				publisher_clock__.stop();
				publisher_clock__.setPeriod( ros::Duration( static_cast<double>(__config.rate) ), false );
			}
			capture_params__.auto_exposure = __config.auto_exposure;
			capture_params__.exposure_time = static_cast<unsigned int>(__config.exposure_time);
			capture_params__.dense_cloud = __config.dense_cloud;

			run_mode__ = __config.run_mode;

			grab_locker__.lock();
			camera__->configureCapture(capture_params__);
			grab_locker__.unlock();

			if(run_mode__ == PUBLISHER_MODE )
			{
				ROS_INFO("EnsensoNxNode: Mode changed to PUBLISHER_MODE");
				publisher_clock__.start();
			}
			else
			{
				ROS_INFO("EnsensoNxNode: Mode changed to SERVER_MODE");
			}
		}
		else
		{
			ROS_WARN("EnsensoNxNode ensenso_server service: Camera hasn't been initiliazed, call set_camera to enable it");
		}
		break;
	}
	case 2:
	{
		frame_name__ = __config.frame_name;
		ROS_INFO_STREAM("EnsensoNxNode: Frame ID in cloud message changed to [\x1B[37m" << frame_name__ << "\x1B[0m" << "]");
		break;
	}
	default:
	{
		ROS_WARN("Dynamic reconfigure callback ran but no reconfiguration was done");
		break;
	}
	}

}

bool EnsensoNxNode::setCameraEnable(std_srvs::SetBoolRequest& __req, std_srvs::SetBoolResponse& __res)
{
	// if called again, and the camera is already opened, it will destroy/close and reopen again
	grab_locker__.lock();
	ROS_WARN("deb1");
	if( __req.data && !is_camera_enabled__ )
	{
		ROS_WARN("deb2");
		camera__.reset(new Device(serial_number__));
		ROS_WARN("deb3");
		if( camera__ )
		{
			ROS_WARN("configure capture");
			camera__->configureCapture(capture_params__);
			//ROS_WARN("configure HE");
			//camera__->configureHECal(he_cal_params__);
			ROS_WARN("End configure ");
			is_camera_enabled__ = true;
			__res.message = "EnsensoNx is Ready";
		}
	}
	else
	{
		camera__.reset();
		is_camera_enabled__ = false;
		__res.message = "EnsensoNx is closed";
	}
	grab_locker__.unlock();
	__res.success = true;
	return true;
}

bool EnsensoNxNode::grabCloud()
{
	if(!is_camera_enabled__)
	{
		ROS_WARN("EnsensoNxNode ensenso_server service: Camera hasn't been initiliazed, call set_camera to enable it");
		return false;
	}

  std::cout << "Inside grabCloud!" << std::endl;
	int good_result = 1;
	int result = camera__->capture(cloud__);
	if ( result == good_result )
	{
		ros::Time ts = ros::Time::now();
		cloud__.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
		cloud__.header.frame_id = frame_name__;
		return true;
	}
	else
	{
		ROS_WARN("EnsensoNxNode::capture(): Error with point cloud capture");
		return false;
	}
}

bool EnsensoNxNode::grabCloudGrayScale()
{
	if(!is_camera_enabled__)
	{
		ROS_WARN("EnsensoNxNode ensenso_server service: Camera hasn't been initiliazed, call set_camera to enable it");
		return false;
	}

	std::cout <<"inside grab grayscale capturing" << std::endl;

	int good_result = 1;
	std::cout <<"graydeb1" << std::endl;
	int result = camera__->capture(cloud_grayscale__);
	std::cout <<"captured" << std::endl;
	if ( result == good_result )
	{
		ros::Time ts = ros::Time::now();
		cloud_grayscale__.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
		cloud_grayscale__.header.frame_id = frame_name__;
		return true;
	}
	else
	{
		ROS_WARN("EnsensoNxNode::capture(): Error with point cloud capture");
		return false;
	}
}

bool EnsensoNxNode::grabCloudRGB()
{
	if(!is_camera_enabled__)
	{
		ROS_WARN("EnsensoNxNode ensenso_server service: Camera hasn't been initiliazed, call set_camera to enable it");
		return false;
	}

	int good_result = 1;
	int result = camera__->capture(cloud_rgb__);
	if ( result == good_result )
	{
		ros::Time ts = ros::Time::now();
		cloud_rgb__.header.stamp = (pcl::uint64_t)(ts.toSec()*1e6); //TODO: should be set by the EnsensoNx::Device class
		cloud_rgb__.header.frame_id = frame_name__;
		return true;
	}
	else
	{
		ROS_WARN("EnsensoNxNode::capture(): Error with point cloud capture");
		return false;
	}
}

void EnsensoNxNode::publisherCallBack(const ros::TimerEvent& __e)
{
	std::cout << "PUBLISHERRR" << std::endl;
	if( grab_locker__.try_lock() )
	{
		bool result = grabCloud();
		pcl::PointCloud<pcl::PointXYZ> cloud = cloud__;
		grab_locker__.unlock();
		if( result )
		{
			sensor_msgs::PointCloud2 cloud_msg;
			pcl::toROSMsg(cloud, cloud_msg); //see pcl-ros conversions at http://wiki.ros.org/action/fullsearch/pcl/Overview
			cloud_publisher__.publish(cloud_msg);
		}
	}
}

bool EnsensoNxNode::pointCloudServiceCallback(sensor_msgs::SnapshotCloud::Request  & __req, sensor_msgs::SnapshotCloud::Response & __res)
{
	//configure capture according request
	//TO DO !! Setting exposure just before capture does not work prperly .
	/*
	if (_request.exposure == 0)
	{
	capture_params_.auto_exposure_ = true;
	}
	else
	{
	capture_params_.auto_exposure_ = false;
	capture_params_.exposure_time_ = _request.exposure;
	}*/

  std::cout << "callback service has recieved a cloud call!" << std::endl;
	capture_params__.dense_cloud = __req.dense_cloud;
  //capture_params__.flex_view = 0;
	cloud__.clear();
	grab_locker__.lock();
	bool result = grabCloud();
	pcl::PointCloud<pcl::PointXYZ> cloud = cloud__;
	grab_locker__.unlock();
	if( result )
	{
		pcl::toROSMsg(cloud, __res.cloud); //see pcl-ros conversions at http://wiki.ros.org/action/fullsearch/pcl/Overview
	}
	else
	{
		std::cout << "EnsensoNxNode::pointCloudServiceCallback(): Error while capturing point cloud" << std::endl;
	}
	return true;

}

void EnsensoNxNode::advancedSnapshotCallback(const sensor_msgs::AdvancedSnapshotCloudGoalConstPtr &__goal )
{
	std::cout << "iNSIDE advancedSnapshotCallback" << std::endl;

	sensor_msgs::AdvancedSnapshotCloudResult res;
	capture_params__.dense_cloud = __goal->dense_cloud;
	std::cout << "ensenso_nx: Call recieved!  dense? " << capture_params__.dense_cloud << std::endl;

	if (__goal->raw_data == sensor_msgs::AdvancedSnapshotCloudGoal::GRAYSCALE){
		std::cout << "ensenso_nx: grayscale type called! "<< std::endl;
		cloud_grayscale__.clear();
		grab_locker__.lock();
		bool result = grabCloudGrayScale();
		pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
				//cloud_grayscale__;
		grab_locker__.unlock();
		if( result ){
			cloud_rgb.is_dense = cloud_grayscale__.is_dense;
			cloud_rgb.width = cloud_grayscale__.width;
			cloud_rgb.height = cloud_grayscale__.height;
			cloud_rgb.header = cloud_grayscale__.header;
			cloud_rgb.sensor_origin_ = cloud_grayscale__.sensor_origin_;
			cloud_rgb.sensor_orientation_ = cloud_grayscale__.sensor_orientation_;
			cloud_rgb.points.resize(cloud_grayscale__.points.size());
			for (size_t i = 0; i < cloud_grayscale__.points.size(); i++)
			{
				cloud_rgb.points[i].x = cloud_grayscale__.points[i].x;
				cloud_rgb.points[i].y = cloud_grayscale__.points[i].y;
				cloud_rgb.points[i].z = cloud_grayscale__.points[i].z;
				cloud_rgb.points[i].r = static_cast<unsigned int>(cloud_grayscale__.points[i].intensity);
				cloud_rgb.points[i].g = static_cast<unsigned int>(cloud_grayscale__.points[i].intensity);
				cloud_rgb.points[i].b = static_cast<unsigned int>(cloud_grayscale__.points[i].intensity);
				//std::cout << " " << static_cast<unsigned int>(cloud_grayscale__.points[i].intensity);
			}
			//std::cout << std::endl;
			pcl::PointCloud<pcl::PointXYZ> cloud;
			pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZ>(cloud_rgb,cloud);

			pcl::toROSMsg(cloud, res.cloud);
			pcl::toROSMsg(cloud_rgb, res.image);

			snapshot_action__->setSucceeded(res);
		}
		else
		{
			std::cout << "EnsensoNxNode::pointCloudServiceCallback(): Error while capturing point cloud" << std::endl;
		}
	}
	else if (__goal->raw_data == sensor_msgs::AdvancedSnapshotCloudGoal::RGB)
	{
		std::cerr << "For the moment there is no rgb implementation, returning just pointcloud data" << std::endl;
		cloud__.clear();
		grab_locker__.lock();
		bool result = grabCloudRGB();
		pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb = cloud_rgb__;
		grab_locker__.unlock();
		if( result ){
			pcl::toROSMsg(cloud_rgb, res.cloud);
			pcl::toROSMsg(cloud_rgb, res.image);
			snapshot_action__->setSucceeded(res);
		}
		else
		{
			std::cout << "EnsensoNxNode::pointCloudServiceCallback(): Error while capturing point cloud" << std::endl;
		}
	}
	else
	{
		std::cout << "ensenso_nx: no image type called! "<< std::endl;

		cloud__.clear();
		grab_locker__.lock();
		bool result = grabCloud();
		pcl::PointCloud<pcl::PointXYZ> cloud = cloud__;
		grab_locker__.unlock();
		if( result ){
			pcl::toROSMsg(cloud, res.cloud);
			snapshot_action__->setSucceeded(res);
		}
		else
		{
			std::cout << "EnsensoNxNode::pointCloudServiceCallback(): Error while capturing point cloud" << std::endl;
		}
	}
	return;

}

void EnsensoNxNode::HandsEyeCalibrationCallback(const ensenso_nx::HECalibrationGoalConstPtr &__goal)
{
	std::cout << "Inside callback!!!!!!!!: " << std::endl;
	ensenso_nx::HECalibrationResultPtr res;

	camera__->HandsEyeCalibration(__goal,res);



}

}
