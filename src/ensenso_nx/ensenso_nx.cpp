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

	NxLibCommand (cmdCapture).execute();
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

	int ww, hh;
	float px;
	std::vector<float> raw_points;
	std::vector<float> raw_img;
	std::vector<std::vector<uint8_t>> raw_img_r;
	std::vector<std::vector<uint8_t>> raw_img_l;
	int nx_return_code;

	std::cout << "inside capture" << std::endl;

	try
	{

		NxLibCommand cam(cmdCapture);
		cam.parameters()[itmTimeout] = 25000;
		cam.execute();


	}catch (NxLibException &e)
	{

		//std::cerr << "Error in capture: "<< e << std::endl;
		ensensoExceptionHandling (e, "Capture");
		std::cerr << "error code: "<< e.getErrorCode() << std::endl;

	}

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
	std::cout << "photos set: " << photos_set<< std::endl;
	if (!flexview_enabled__)
	{
		photos_set = 1;
		std::cout << "flexview not enabled" << std::endl;
		raw_img_r.resize(1);
		raw_img_l.resize(1);
		camera__[itmImages][itmRaw][itmLeft].getBinaryData(raw_img_l[0], 0);
		camera__[itmImages][itmRaw][itmRight].getBinaryData(raw_img_r[0], 0);
	}
	else
	{

		std::cout << "flexview enabled" << std::endl;
		raw_img_r.resize(photos_set);
		raw_img_l.resize(photos_set);
		for (int i = 0; i < photos_set; i++)
		{

			camera__[itmImages][itmRaw][i][itmLeft].getBinaryData(&nx_return_code, raw_img_l[i], 0);
			camera__[itmImages][itmRaw][i][itmRight].getBinaryData(&nx_return_code, raw_img_r[i], 0);

		}

	}
	std::cout << " Raw img" << std::endl ;
	raw_img.resize((unsigned int)ww*(unsigned int)hh);
	std::cout << " pre for loop" << std::endl ;
	for (int i = 0; i < raw_img.size(); i++)
	{
		std::cout << " inside for loop" << std::endl ;

		int residual = i % 4;
		int ii = (i-residual)*4;
		raw_img[i] = 0;
		std::cout << " pre for loop 2" << std::endl ;
		for (int j=0; j< photos_set/4; j++)
		{
			std::cout << " inside for loop 2 "  << raw_img_l[j*4 + residual].size() <<"  and   :  " << raw_img_l.size() <<"  and   :  " << ii <<std::endl ;
			raw_img[i] += raw_img_l[j*4 + residual][ii] + raw_img_r[j*4 + residual][ii];
			std::cout << " inside for loop 2 end " << j << std::endl ;
			if (j == 0)
			{

				std::cout << " " << raw_img[i];
				std::cout << "." << static_cast<unsigned int>(raw_img_l[j*4 + residual][ii]);

			}

		}
					std::cout << " dividing" << std::endl ;
		raw_img[i] = raw_img[i]/8.0f;
		std::cout << "," << raw_img[i];
					std::cout << " inside for loop 1 end" << std::endl ;



	}
	/*
	std::cout << " Raw img" << std::endl ;
	for (int i = 0; i< raw_img.size();i++)
		std::cout << " " << raw_img[i];
	std::cout << std::endl ;
*/
	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
	std::cout << "entering for. raw_img size: "<< raw_img_l[1].size() <<"	points size: " << raw_points.size() << "h*w" << _p_cloud.width*_p_cloud.height << std::endl;
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
				_p_cloud.points.at(kk).intensity = raw_img[kk];
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
		std::cout << "outside" << std::endl;


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

	camera__[itmParameters][itmCapture][itmTriggerDelay] = 8000.0f;
	if (capture_params__.flex_view < 2)
	{

		flexview_enabled__ = false;
		camera__[itmParameters][itmCapture][itmFlexView] = flexview_enabled__;
		std::cout << "FLEXVIEW DISABLED" << std::endl;

	}
	else
	{

		flexview_enabled__ = true;
		camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);
		std::cout << "FLEXVIEW ENABLED" << std::endl;

	}

}

}
