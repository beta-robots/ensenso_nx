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

  std::cout << "Inside capture xiz" << std::endl;
	int ww, hh;
	float px;
	std::vector<float> raw_points;
	int nx_return_code;

  std::cout << "Executing" << std::endl;

  NxLibCommand cam(cmdCapture);
  cam.parameters()[itmTimeout] = 25000;
  cam.execute();
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();
  std::cout << "Executed" << std::endl;


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
	std::cout << "width 3d: " << ww << "	2d: " << www << std::endl;
	std::cout << "height 3d: " << hh << "	2d: " << hhh << std::endl;
   std::cout << "raw img size: " << raw_img.size() << std::endl;
  std::cout << "raw img size1: " << raw_img_l.size() << std::endl;

  std::cout << "raw point size: " << raw_points.size() << std::endl;
	raw_img.resize((unsigned int)ww*(unsigned int)hh);



	/*for (int i = 0; i < raw_img.size(); i++)
	{

		int residual = i % 4;
		int ii = (i-residual);
		raw_img[i] = 0;
		for (int j=0; j< photos_set/4; j++)
		{
			//raw_img[i] += raw_img_l[j*4 + residual][ii] + raw_img_r[j*4 + residual][ii];
		}

		//raw_img[i] = raw_img[i]/(2.0f);


	}*/

  for (int i = 0; i < raw_img.size(); i++)
  {
raw_img[i] = 0;

    for (int j = 0; j < raw_img_l.size(); j++)
		{



        raw_img[i] = raw_img[i]  + raw_img_l[j][i];

        //raw_img[i] = raw_img[i]  + (raw_img_l[j][i] + raw_img_r[j][i])/2;



    }
    raw_img[i] = raw_img[i]/raw_img_l.size();

  }
//std::vector<std::vector<float>> kernel = {{0.0f,0.0f,2.0f,0.0f,0.0f},{0.0f,3.0f,4.0f,3.0f,0.0f},{2.0f,4.0f,0.0f,4.0f,2.0f},{0.0f,3.0f,4.0f,3.0f,0.0f},{0.0f,0.0f,2.0f,0.0f,0.0f}};
  //std::vector<std::vector<float>> kernel = {{0.0f,0.0f,2.0f,0.0f,0.0f},{0.0f,3.0f,5.0f,3.0f,0.0f},{2.0f,5.0f,10.0f,5.0f,2.0f},{0.0f,3.0f,5.0f,3.0f,0.0f},{0.0f,0.0f,2.0f,0.0f,0.0f}};
  std::vector<std::vector<float>> kernel_sharpen0 = {{0.0f,-1.0f,0.0f},{-1.0f,5.0f,-1.0f},{0.0f,-1.0f,0.0f}};//sharpen
  std::vector<std::vector<float>> kernel_sharpen = {{-1.0f,-1.0f,-1.0f},{-1.0f,9.0f,-1.0f},{-1.0f,-1.0f,-1.0f}};//sharpen
  std::vector<std::vector<float>> kernel_sharpen2 = {{-2.0f,-2.0f,-2.0f},{-2.0f,18.0f,-2.0f},{-2.0f,-2.0f,-2.0f}};//sharpen
  std::vector<std::vector<float>> kernel_sharpen3 = {{0.0f,0.0f,-1.0f,0.0f,0.0f},{0.0f,-1.0f,-2.0f,-1.0f,0.0f},{-1.0f,-2.0f,17.0f,-2.0f,-1.0f},{0.0f,-1.0f,-2.0f,-1.0f,0.0f},{0.0f,0.0f,-1.0f,0.0f,0.0f}};//sharpen
  std::vector<std::vector<float>> kernel_blur = {{1.0f/16.0f,1.0f/16.0f,1.0f/16.0f},{1.0f/16.0f,1.0f/2.0f,1.0f/16.0f},{1.0f/16.0f,1.0f/16.0f,1.0f/16.0f}};//common blur
  std::vector<std::vector<float>> kernel_outline = {{-1.0f,-1.0f,-1.0f},{-1.0f,8.0f,-1.0f},{-1.0f,-1.0f,-1.0f}};//outline
  std::vector<std::vector<float>> kernel_hzline= {{-1.0f/6.0f,-1.0f/6.0f,-1.0f/6.0f},{1.0f/3.0f,1.0f/3.0f,1.0f/3.0f},{-1.0f/6.0f,-1.0f/6.0f,-1.0f/6.0f}};// hline
  std::vector<std::vector<float>> kernel_vline_fill = {{1.0f/7.0f,0.0f,1.0f/7.0f},{1.0f/7.0f,1.0f/7.0f,1.0f/7.0f},{1.0f/7.0f,1.0f/7.0f,1.0f/7.0f}};//outline
  //std::vector<std::vector<float>> kernel = {{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f}};
//for (int k = 0; k < 3; k++)
//    convolutionalFiltering(raw_img,ww,hh,kernel_blur);
//  convolutionalFiltering(raw_img,ww,hh,kernel_sharpen);
//convolutionalFiltering(raw_img,ww,hh,kernel_hzline);
//    convolutionalFiltering(raw_img,ww,hh,kernel_sharpen);
//convolutionalFiltering(raw_img,ww,hh,kernel_sharpen);

/*

	std::cout << " Raw img" << std::endl ;
  for (int i = 500; i< 900; i++)
		std::cout << " " << raw_img[i];
	std::cout << std::endl ;
*/
	//Move raw data to point cloud
	_p_cloud.width = (unsigned int)ww;
	_p_cloud.height = (unsigned int)hh;
	_p_cloud.resize(_p_cloud.width*_p_cloud.height);
	unsigned int kk = 0;
//	std::cout << "entering for. raw_img size: "<< raw_img_l[1].size() <<"	points size: " << raw_points.size() << "h*w" << _p_cloud.width*_p_cloud.height << std::endl;
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

	/*std::cout << "Distances!" << std::endl;
	for (int i =0 ;i <_p_cloud.points.size();i++)
	{

		if (i > 10 )
		{

			float module = sqrt(pow(_p_cloud.points.at(i).x - _p_cloud.points.at(i-1).x,2) + pow(_p_cloud.points.at(i).y - _p_cloud.points.at(i-1).y,2)+pow(_p_cloud.points.at(i).z-_p_cloud.points.at(i-1).z,2));
			std::cout << " " << module;
		}


	}
	std::cout <<  std::endl;
	*/

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
		std::cout << "FLEXVIEW DISABLED" << std::endl;

	}
	else
	{

		flexview_enabled__ = true;
		camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);
    //camera__[itmParameters][itmCapture][itmHdr] = true;
    //camera__[itmParameters][itmCapture][itmGainBoost] = true;
		std::cout << "FLEXVIEW ENABLED" << std::endl;

  }


  if (capture_params__.flex_view > 8)
      camera__[itmParameters][itmDisparityMap][itmStereoMatching][itmMethod] = "Correlation";





}

void Device::convolutionalFiltering(std::vector<float>& __2d_image,const int __width,const int __height,const std::vector<std::vector<float>> __kernel,int iter_n)
{



    std::vector<float> _2d_image_copy = __2d_image;

    if (__kernel.size()%2 == 0)
    {
      std::cerr << "Error filtering the 2d data! kernel width must have a impar number to respect the simmetry!" << std::endl;
      return;
    }

    if (__kernel.size()%2 == 0)
    {
      std::cerr << "Error filtering the 2d data! kernel width must have a impar number to respect the simmetry!" << std::endl;
      return;
    }



    int kernel_w = (__kernel.size()-1)/2;
    int kernel_h = (__kernel[0].size()-1)/2;

    //Normalising kernel
    /*
    int module_val = 0;
    for (size_t i = 0; i< __kernel.size(); i++)
    {
      for (size_t j = 0; j< __kernel[0].size(); j++)
      {
      module_val += abs(__kernel[i][j]);
      }
    }

  std::vector<std::vector<float>> normalised_kernel = __kernel;
    for (size_t i = 0; i< __kernel.size(); i++)
    {
      for (size_t j = 0; j< __kernel[0].size(); j++)
      {
      normalised_kernel[i][j] = __kernel[i][j]/module_val;
      }
    }
    */
    std::vector<std::vector<float>> normalised_kernel = __kernel;



    std::cout << "wwww: " << __width << std::endl;
    std::cout << "hhh" << __height << std::endl;
    std::cout << "max " << __width*__height << std::endl;

    for (size_t j = 0; j< __height; j++)
    {
      for (size_t i = 0; i< __width; i++)
      {



        //Conditions to allow the filtering of a pixel
        bool condition_1 = (i - kernel_w)>= 0;
        bool condition_2 = (i + kernel_w)<= __width;
        bool condition_3 = (j - kernel_h)>= 0;
        bool condition_4 = (j + kernel_h)< __height;

        if (condition_1 && condition_2 && condition_3 && condition_4)
        {

          //Its okay! lets filter the pixel!
          __2d_image[i + j*__width] = 0.0f;
          for (int ii = i - kernel_w; ii<=  i + kernel_w; ii++)
          {

            for (int jj = j - kernel_h; jj<=  j + kernel_h; jj++)
            {

              __2d_image[i + j*__width] += _2d_image_copy[ii + jj*__width]*normalised_kernel[ii-(i - kernel_w)][jj-(j - kernel_h)];

              if (i == 8 && j == 8)
              {

                std::cout << "i: " << i << std::endl;
                std::cout << "j: " << j << std::endl;
                std::cout << "ii: " << ii << std::endl;
                std::cout << "jj: " << jj << std::endl;
                std::cout << "ii k: " << (ii-(i - kernel_w)) << std::endl;
                std::cout << "jj k: " << (jj-(j - kernel_h)) << std::endl;
                std::cout << "kernel val: " << normalised_kernel[ii-(i - kernel_w)][jj-(j - kernel_h)] << std::endl;
                std::cout << "pre val: " << _2d_image_copy[i + j*__width] << std::endl;
                std::cout << "post val : " << __2d_image[i + j*__width] << std::endl;
                std::cout <<  std::endl;
                std::cout <<  std::endl;

              }

            }

          }

        }

      }

    }




}


}
