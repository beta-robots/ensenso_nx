#include <ensenso_nx/ensenso_nx.h>

namespace ensenso_nx
{

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
	std::vector<int> raw_img;
	int nx_return_code;

	NxLibCommand (cmdCapture).execute();
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
				_p_cloud.points.at(kk).intensity = raw_img[(ii*_p_cloud.width + jj)];
				kk++;
			}
			else //in case of nan, check dense_cloud_ to fill in the cloud or not
			{
				if (capture_params__.dense_cloud)
				{
					_p_cloud.points.at(kk).x = std::nan("");
					_p_cloud.points.at(kk).y = std::nan("");
					_p_cloud.points.at(kk).z = std::nan("");
					_p_cloud.points.at(kk).intensity = std::nan("");
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

	NxLibCommand (cmdCapture).execute();
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
					_p_cloud.points.at(kk).r = std::nan("");
					_p_cloud.points.at(kk).g= std::nan("");
					_p_cloud.points.at(kk).b= std::nan("");
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

    NxLibItem flexViewNode = camera__[itmParameters][itmCapture][itmFlexView];
    if (flexViewNode.exists())
    {
        camera__[itmParameters][itmCapture][itmFlexView] = capture_params__.flex_view;
        if (capture_params__.flex_view < 2)
        {
            camera__[itmParameters][itmCapture][itmFlexView] = false;
        }
        else
        {
            flexview_enabled__ = true;
        }
    }
}

}
