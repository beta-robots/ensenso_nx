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

Device::Device(const std::string & __serial_num)
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

	NxLibCommand open(cmdOpen);
	open.parameters()[itmCameras] = device_params__.serial_num;
	open.execute();
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

	for (int i = 0; i < raw_img.size(); i++)
	{
		raw_img[i] = 0;

		for (int j = 0; j < raw_img_l.size(); j++)
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

}
