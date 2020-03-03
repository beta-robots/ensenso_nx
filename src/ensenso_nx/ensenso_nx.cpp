#include <ensenso_nx/ensenso_nx.h>


#define LOG_NXLIB_EXCEPTION(EXCEPTION)
namespace ensenso_nx
{

void ensensoExceptionHandling (const NxLibException &ex,
		 std::string func_nam)
{
	PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
	 ex.getItemPath ().c_str ());
	if (ex.getErrorCode () == NxLibExecutionFailed)
	{
		NxLibCommand cmd ("");
		PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
	}
}

Device::Device(const std::string & __serial_num):
	free_mode__("motion_server/free_mode", true)
{
	std::cout << "EnsensoNx::Device: Opening camera ..." << std::endl;

	nxLibInitialize(true);

	// Create an object referencing the camera's tree item, for easier access:
	camera__ = nx_lib_root__[itmCameras][itmBySerialNo][__serial_num];
	if (!camera__.exists() || (camera__[itmType] != valStereo))
	{
		std::cout << "EnsensoNx::Device: Camera not found. Please connect a single stereo camera to your computer" << std::endl;
		return;
	}
	device_params__.serial_num = camera__[itmSerialNumber].asString();
	//tf_listener_ptr__.reset(new tf2_ros::TransformListener(tf2_buffer__));

	NxLibCommand open(cmdOpen);
	open.parameters()[itmCameras] = device_params__.serial_num;
	open.execute();
	free_mode__.waitForServer();
	std::cout << "EnsensoNx::Device: Camera open. SN: " << device_params__.serial_num << std::endl;
}

Device::~Device()
{
	std::cout << "EnsensoNx::Device: Closing camera ..." << std::endl;
	NxLibCommand close(cmdClose);
	close.parameters()[itmCameras] = device_params__.serial_num;
	close.execute();
	std::cout << "EnsensoNx::Device: Camera closed. SN: " << device_params__.serial_num << std::endl;
	nxLibFinalize();
}

void Device::configureCapture(const CaptureParams & __params)
{
	capture_params__.auto_exposure = __params.auto_exposure;
	capture_params__.exposure_time = __params.exposure_time;
	capture_params__.dense_cloud = __params.dense_cloud;
	capture_params__.flex_view = __params.flex_view;
	configureCapture();
}

void Device::configureHECal(const HECalParams & __params)
{
	he_cal_params__.decode_data = __params.decode_data;
	he_cal_params__.grid_spacing = __params.grid_spacing;

}

// void Device::configureExposure(unsigned int _exposure)
// {
//     if (_exposure == 0) //autoexposure case
//     {
//         capture_params_.auto_exposure_ = true;
//     }
//     else //manual exposure case
//     {
//         capture_params_.auto_exposure_ = false;
//         capture_params_.exposure_time_ = _exposure;
//     }
//
//     //call protected member to set the configuration to the camera
//     this->configureCapture();
// }

int Device::capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{

	int ww, hh;
	float px;
	std::vector<float> raw_points;
	int nx_return_code;

	NxLibCommand cam(cmdCapture);
	cam.parameters()[itmTimeout] = 25000;
	cam.execute();
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();

	// Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}



int Device::capture(pcl::PointCloud<pcl::PointXYZI> & _p_cloud)
{

	int ww, hh,www,hhh;
	float px;
	std::vector<float> raw_points;
	std::vector<float> raw_img;
  
  //raw_img_r  is not used for the moment. The pointcloud is focused in left_lens by default, so right one is hard to use. Nevertheles, here it is!
	std::vector<std::vector<uint8_t>> raw_img_r;
	std::vector<std::vector<uint8_t>> raw_img_l;
	int nx_return_code;

	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();

	//Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Get 2D image raw data
	raw_img_r.clear();
	raw_img_l.clear();
	int photos_set = capture_params__.flex_view;
	photos_set = photos_set - (photos_set % 4);
	if (!flexview_enabled__)
	{
		photos_set = 1;
		raw_img_r.resize(1);
		raw_img_l.resize(1);
		camera__[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&www, &hhh, 0,0,0,0);
		camera__[itmImages][itmRectified][itmLeft].getBinaryData(raw_img_l[0], 0);
		camera__[itmImages][itmRectified][itmRight].getBinaryData(raw_img_r[0], 0);
	}
	else
	{
		camera__[itmImages][itmRectified][0][itmLeft].getBinaryDataInfo(&www, &hhh, 0,0,0,0);
		raw_img_r.resize(photos_set);
		raw_img_l.resize(photos_set);
		for (int i = 0; i < photos_set; i++)
		{

      camera__[itmImages][itmRectified][i][itmLeft].getBinaryData(&nx_return_code, raw_img_l[i], 0);
      camera__[itmImages][itmRectified][i][itmRight].getBinaryData(&nx_return_code, raw_img_r[i], 0);

		}

	}
	raw_img.resize((unsigned int)ww*(unsigned int)hh);

	for (size_t i = 0; i < raw_img.size(); i++)
	{
		raw_img[i] = 0;

		for (size_t j = 0; j < raw_img_l.size(); j++)
		{

				raw_img[i] = raw_img[i]  + raw_img_l[j][i];



		}
		raw_img[i] = raw_img[i]/raw_img_l.size();

	}

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				_p_cloud.points.at(kk).intensity = raw_img[(ii*_p_cloud.width + jj)];//raw_img[kk];
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					//std::cout << "for some reason we are entering here at index " << kk << std::endl;
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					_p_cloud.points.at(kk).intensity = 0;
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}

int Device::capture(pcl::PointCloud<pcl::PointXYZRGB> & _p_cloud)
{

	int ww, hh;
	float px;
	std::vector<float> raw_points;
	std::vector<int> raw_img;
	int nx_return_code;



		NxLibCommand(cmdCapture).execute();
		NxLibCommand (cmdComputeDisparityMap).execute();
		NxLibCommand (cmdComputePointMap).execute();




	//Get image dimensions
	camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);

	//Get 3D image raw data
	camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);

	//Get 2D image raw data
	if (!flexview_enabled__)
	{
		camera__[itmImages][itmRaw][itmLeft].getBinaryData(&nx_return_code, raw_img, 0);
	}
	else
	{
		camera__[itmImages][itmRaw][0][itmLeft].getBinaryData(&nx_return_code, raw_img, 0);
	}

	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
	{
		for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
		{
			px = raw_points[(ii*_p_cloud.width + jj)*3];
			if ( !std::isnan(px) )
			{
				_p_cloud.points.at(kk).x = px/1000.;
				_p_cloud.points.at(kk).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
				_p_cloud.points.at(kk).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
				_p_cloud.points.at(kk).r = raw_img[(ii*_p_cloud.width + jj)*3];
				_p_cloud.points.at(kk).g = raw_img[(ii*_p_cloud.width + jj)*3+1];
				_p_cloud.points.at(kk).b = raw_img[(ii*_p_cloud.width + jj)*3+2];
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					_p_cloud.points.at(kk).r = 0;
					_p_cloud.points.at(kk).g= 0;
					_p_cloud.points.at(kk).b= 0;
					kk++;
				}
				else
				{
					//nothing to do , the point is lost
				}
			}
		}
	}

	//resize with number valid points. If _dense_cloud, just set the flag ordered to true
	_p_cloud.resize(kk);//checks if kk=ww*hh to set the cloud as ordered (width,height) or unordered (width=size,height=1)
	_p_cloud.is_dense = capture_params__.dense_cloud;

	//debug message
	//     std::cout << "Cloud capture: " << std::endl <<
	//                  "\treturn code: " << nx_return_code << std::endl <<
	//                  "\tnum points: " << raw_points.size()/3 << std::endl <<
	//                  "\twidth: " << ww << std::endl <<
	//                  "\theight: " << hh << std::endl <<
	//                  "\tvalid_points: " << kk << std::endl;

	//return success
	return 1;
}

void Device::configureCapture()
{

	camera__[itmParameters][itmCapture][itmAutoExposure] = capture_params__.auto_exposure;
	camera__[itmParameters][itmCapture][itmExposure    ] = static_cast<double>(capture_params__.exposure_time); //TODO check if requires cast to double.


	if (capture_params__.flex_view < 2)
	{

		flexview_enabled__ = false;
		camera__[itmParameters][itmCapture][itmFlexView] = flexview_enabled__;

	}
	else
	{

		flexview_enabled__ = true;
		camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);
		//camera__[itmParameters][itmCapture][itmHdr] = true;
		//camera__[itmParameters][itmCapture][itmGainBoost] = true;

	}

	if (capture_params__.flex_view > 8)
			camera__[itmParameters][itmDisparityMap][itmStereoMatching][itmMethod] = "Correlation";


}



int Device::HandsEyeCalibration(const ensenso_nx::HECalibrationGoalConstPtr &__goal, const ensenso_nx::HECalibrationResultConstPtr &__result)
{

	//MISSING CHECK IF A PATTERN IS OBTAINED OR NOT. IN CASE IT IS NOT OBTAINED, REPEAT.

	// will need to adapt this line to the size of the calibration pattern that you are using.
	// Discard any pattern observations that might already be in the pattern buffer.
	NxLibCommand(cmdDiscardPatterns).execute();
	NxLibCommand decode(cmdCollectPattern);
	motion_server::FreeModeGoal goal_free_mode;
	goal_free_mode.until_button_pressed.data = true;
	int nx_return_code;
	double timestamp;
	if (!he_cal_params__.decode_data)
	{
			camera__[itmParameters][itmPattern][itmGridSpacing] = he_cal_params__.grid_spacing;
	}
	else
	{
			decode.parameters()[itmDecodeData] = true;
			decode.execute();
	}



	// Turn off the camera's projector so that we can observe the calibration pattern.
	camera__[itmParameters][itmCapture][itmProjector] = false;
	camera__[itmParameters][itmCapture][itmFrontLight] = true;

	// You can adapt this number depending on how accurate you need the
	// calibration to be.
	for (int i = 0; i < __goal->position_number; i++) {
			// Move your robot to a new position from which the pattern can be seen. It might be a good idea to
			free_mode__.sendGoal(goal_free_mode);
			std::cout << "Waiting to confirm new position" << std::endl;
			free_mode__.waitForResult();
			std::cout << "New position have been declared" << std::endl;

			//Make sure that the robot is not moving anymore. You might want to wait for a few seconds to avoid
			//any oscillations.
			sleep(2);

			// Observe the calibration pattern and store the observation in the pattern buffer.
			NxLibCommand capture(cmdCapture);
			capture.parameters()[itmCameras] = device_params__.serial_num;
			capture.execute();
			bool foundPattern = false;
			try {
				std::vector<std::vector<float>> matrix_t;
					NxLibCommand collectPattern(cmdCollectPattern);
					collectPattern.parameters()[itmCameras] = device_params__.serial_num;
					collectPattern.execute();

					collectPattern.result()[itmStereo][itmLeft][itmTransformation].getBinaryData(&nx_return_code,matrix_t,&timestamp);

					std::cout << "Matrix: " << std::endl;
					for (size_t i = 0; i< matrix_t.size(); i++)
					{
						for (size_t j = 0; j< matrix_t[i].size(); j++)
						{
							std::cout << matrix_t[i][j] << " ";
						}
						std::cout << std::endl;
					}
					foundPattern = true;
			} catch (NxLibException&) {}

			if (foundPattern) {
					// We actually found a pattern. Get the current pose of your robot (from which the pattern was
					// observed) and store it somewhere.
					geometry_msgs::TransformStamped table_to_ensenso = tf2_buffer__.lookupTransform("table", "world", ros::Time(0), ros::Duration(0.5));
			} else {
					// The calibration pattern could not be found in the camera image. When your robot poses are
					// selected randomly, you might want to choose a different one.
			}
	}

	// You can insert a recalibration here, as you already captured stereo patterns anyway. See here for a
	// code snippet that does a recalibration.

	// We collected enough patterns and can start the calibration.
	NxLibCommand calibrateHandEye(cmdCalibrateHandEye);
	calibrateHandEye.parameters()[itmSetup] = __goal->type;

	// At this point, you need to put your stored robot poses into the command's Transformations parameter.
	//calibrateHandEye.parameters()[itmTransformations] = ...;

	// Start the calibration. Note that this might take a few minutes if you did a lot of pattern observations.
	calibrateHandEye.execute();

	//calibrateHandEye[itmResult][itmResidual].getBinaryData(&nx_return_code, __result->score, 0);
	std::string return_d;
	std::vector<float> ret;
	calibrateHandEye.result()[itmResidual].getBinaryData(&nx_return_code,ret,&timestamp);

	std::cout << "residual" << std::endl;
	for (size_t i = 0; i < ret.size(); i++)
	{
		std::cout << ret[i]<< " ";

	}
	std::cout << std::endl;

	std::vector<float> transforms;
	calibrateHandEye.result()[itmPatternPose].getBinaryData(&nx_return_code,transforms,&timestamp);
	std::cout << "transform" << std::endl;
	for (size_t i = 0; i < ret.size(); i++)
	{
		std::cout << transforms[i]<< " ";

	}
	std::cout << std::endl;

	// Store the new calibration to the camera's EEPROM.
	/*
	NxLibCommand storeCalibration(cmdStoreCalibration);
	storeCalibration.parameters()[itmCameras] = device_params__.serial_num;
	storeCalibration.parameters()[itmLink] = true;
	storeCalibration.execute();
	*/

	camera__[itmParameters][itmCapture][itmProjector] = true;
	camera__[itmParameters][itmCapture][itmFrontLight] = false;
	return 0;


}

}
