#include <ensenso_nx/ensenso_nx.h>


#define LOG_NXLIB_EXCEPTION(EXCEPTION)
namespace ensenso_nx
{

void ensensoExceptionHandling (const NxLibException &ex,
		 std::string func_nam)
{
	PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str(), ex.getErrorText().c_str (), ex.getErrorCode(),
	 ex.getItemPath().c_str ());
	if (ex.getErrorCode () == NxLibExecutionFailed)
	{
		NxLibCommand cmd ("");
		PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
	}
}

Device::Device(const std::string & __serial_num):
	free_mode__("/motion_server/free_mode", true)
{
	std::cout << "EnsensoNx::Device: Opening camera ..." << std::endl;

	try{
	nxLibInitialize(true);
	}
	catch(NxLibException &ex)
	{
		ensensoExceptionHandling(ex,"Initialize");

	}



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
	std::cout << "EnsensoNx::Device: Camera open." << device_params__.serial_num << std::endl;
	free_mode__.waitForServer(ros::Duration(2.0));

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


	cv::Mat img_cv_raw_t, img_cv_raw_now, img_cv_rect_t, img_cv_rect_now;
	std::vector<cv::Mat> mat_array;
	std::vector<float> exposures;

	int ww, hh,www,hhh;
	float px;
	std::vector<float> raw_points;
	std::vector<float> raw_img;
	std::vector<std::vector<uint8_t>> raw_img_r;
	std::vector<std::vector<uint8_t>> raw_img_l;
	int nx_return_code;



	{
		NxLibCommand cam(cmdCapture);
		//cam.parameters()[itmTimeout] = 25000;
		std::cout << "deb0. True is "<<true << " and false is "<<false << std::endl;



		camera__[itmParameters][itmCapture][itmFlexView] = false;


		try {
			camera__[itmParameters][itmCapture][itmExposure] = 6;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb2");
		}

		try {
			camera__[itmParameters][itmCapture][itmAutoBlackLevel] = true;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb2");
		}

		try {
		camera__[itmParameters][itmCapture][itmAutoExposure] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb4");
		}

		try {
			camera__[itmParameters][itmCapture][itmAutoGain] = true;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb4");
		}

		try {
		camera__[itmParameters][itmCapture][itmBinning] = 1;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb5");
		}

		try {
		camera__[itmParameters][itmCapture][itmBlackLevelOffset] = 0.5;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb6");
		}

		try {
		camera__[itmParameters][itmCapture][itmBlackLevelOffsetCalibration][itmLeft] = 0;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb7");
		}

		try {
		camera__[itmParameters][itmCapture][itmBlackLevelOffsetCalibration][itmRight] = 0;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb8");
		}



		try {
		camera__[itmParameters][itmCapture][itmFlashDelay] = -0.0500000000000000028;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb10");
		}

		try {
		camera__[itmParameters][itmCapture][itmFrontLight] = true;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb12");
		}

		try {
		camera__[itmParameters][itmCapture][itmGain] = 1;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb13");
		}

		try {
		camera__[itmParameters][itmCapture][itmGainBoost] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb14");
		}

		try {
		camera__[itmParameters][itmCapture][itmHardwareGamma] = true;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb15");
		}

		try {
		camera__[itmParameters][itmCapture][itmHdr] = true;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb16");
		}

		try {
		camera__[itmParameters][itmCapture][itmImageBuffer][itmCount] = 0;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb17");
		}

		try {
		camera__[itmParameters][itmCapture][itmImageBuffer][itmOverflowPolicy] = "DiscardOld";
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb18");
		}

		try {
		camera__[itmParameters][itmCapture][itmImageBuffer][itmSize] = 1;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb19");
		}

		try {
		camera__[itmParameters][itmCapture][itmMode] = "Rectified";
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb22");
		}

		try {
		camera__[itmParameters][itmCapture][itmMultiExposureFactor] = 1;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb23");
		}

		try {
		camera__[itmParameters][itmCapture][itmPixelClock] = 24;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb24");
		}

		try {
		camera__[itmParameters][itmCapture][itmProjector] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb25");
		}

		try {
		camera__[itmParameters][itmCapture][itmTargetBrightness] = 80;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb27");
		}


		try {
		camera__[itmParameters][itmCapture][itmTriggerDelay] = 0;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb28");
		}

		try {
		camera__[itmParameters][itmCapture][itmTriggerMode] = "Software";
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb29");
		}

		try {
		camera__[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb31");
		}

		try {
		camera__[itmParameters][itmCapture][itmUseRecalibrator] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"deb32");
		}

		try {
		camera__[itmParameters][itmCapture][itmWaitForRecalibration] = false;
		} catch (NxLibException& ex) {
			ensensoExceptionHandling(ex,"de33");
		}


	}
NxLibCommand (cmdCapture).execute();
usleep(25000);

NxLibCommand (cmdCapture).execute();

double timestamp;
camera__[itmImages][itmRectified][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);

NxLibCommand (cmdRectifyImages).execute();
sleep(1);
camera__[itmImages][itmRectified][itmLeft].getBinaryData(&nx_return_code, img_cv_rect_t, 0);


//	cv::imshow( "1", mat1 );
//	cv::waitKey(0);

//	cv::imshow( "2", mat2 );
//	cv::waitKey(0);

//	cv::imshow( "3", mat3 );
//	cv::waitKey(0);

//	cv::imshow( "4", mat4 );
//	cv::waitKey(0);

//	cv::imshow( "5", mat5 );
//	cv::waitKey(0);

//	cv::imshow( "6", mat6 );
//	cv::waitKey(0);

//	cv::imshow( "7", mat7 );
//	cv::waitKey(0);


//	usleep(1000000);
//	mat_array.push_back(mat1);
//	mat_array.push_back(mat2);
//	mat_array.push_back(mat3);
//	mat_array.push_back(mat4);
//	mat_array.push_back(mat5);
//	mat_array.push_back(mat6);
//	mat_array.push_back(mat7);

//	cv::Mat response;
//	cv::Ptr<cv::CalibrateDebevec> calibrateDebevec = cv::createCalibrateDebevec();
//		usleep(1000000);
//		sleep(2);

//	calibrateDebevec->process(mat_array, response, exposures);

//	std::cout << "deb1" << std::endl;

//	cv::Mat hdr;
//	cv::Ptr<cv::MergeDebevec> merge_debevec = cv::createMergeDebevec();
//	merge_debevec->process(mat_array, img_cv_rect_t, exposures, response);
//	std::cout << "deb2" << std::endl;


//	std::cout << "deb0000" << std::endl;
//	cv::Mat response;
//		std::cout << "deb00001" << std::endl;
//	cv::Ptr<cv::CalibrateRobertson> calibrateRobertson = cv::createCalibrateRobertson();
//		usleep(1000000);
//		sleep(2);
//	std::cout << "deb000021" << std::endl;
//	calibrateRobertson->process(mat_array, response, exposures);

//	std::cout << "deb1" << std::endl;

//	cv::Mat hdr;
//	cv::Ptr<cv::MergeRobertson> merge_robertson = cv::createMergeRobertson();
//	merge_robertson->process(mat_array, hdr, exposures, response);
//	std::cout << "deb2" << std::endl;

//	cv::Mat ldr;
//	cv::Ptr<cv::Tonemap> tonemap = cv::createTonemap(gamma + static_cast<float>(i));
//	tonemap->process(hdr, img_cv_rect_t);

//	cv::Ptr<cv::MergeMertens> merge_mertens = cv::createMergeMertens();
//	merge_mertens->process(mat_array, img_cv_rect_t);
//	std::cout << "deb4" << std::endl;

/*temps*/
//cv::addWeighted(mat1,0.5,mat2,0.5,0,mat11);
//cv::addWeighted(mat3,0.5,mat4,0.5,0,mat12);
//cv::addWeighted(mat5,0.5,mat6,0.5,0,mat13);
//cv::addWeighted(mat7,0.5,mat8,0.5,0,mat14);
//cv::addWeighted(mat9,0.5,mat10,0.5,0,mat15);

//cv::addWeighted(mat11,0.5,mat12,0.5,0,mat21);
//cv::addWeighted(mat13,0.5,mat14,0.5,0,mat22);

//cv::addWeighted(mat21,0.5,mat22,0.5,0,mat31);

//cv::addWeighted(mat31,0.8,mat15,0.2,0,img_cv_rect_t);
/*temps*/

/*			std::cout << "deb35.5" << std::endl;
	try {
	NxLibCommand (cmdComputePointMap).execute();
			} catch (NxLibException& ex) {
				ensensoExceptionHandling(ex,"de35.5");
			}
*//*
			std::cout << "deb36" << std::endl;
	usleep(20000);
			std::cout << "deb37" << std::endl;
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();
			std::cout << "deb38" << std::endl;
	usleep(20000);
			std::cout << "deb39" << std::endl;
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();
	usleep(20000);
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();
	usleep(20000);
	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();
	usleep(20000);
*/
	//Get image dimensions
//			std::cout << "deb1" << std::endl;
//	try {
//	camera__[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&www, &hhh, 0,0,0,0);
//	} catch (NxLibException& ex) {
//		ensensoExceptionHandling(ex,"de36");
//	}



//cv::Mat img_cv_rect_t = cv::Mat::zeros(mat1.size().height, mat1.size().width,CV_32F);
//	//Get 2D image raw data


//	try {

//	camera__[itmImages][itmRectified][itmLeft].getBinaryData(&nx_return_code, img_cv_rect_t, 0);
//	} catch (NxLibException& ex) {
//		ensensoExceptionHandling(ex,"de37");
//	}

//			std::cout << "deb3  " << img_cv_rect_t.size().width << "  "  << img_cv_rect_t.size().height << std::endl;
//			std::cout << "deb33  " << mat1.size().width << "  "  << mat1.size().height << std::endl;

//cv::imshow( "hdr", hdr*255 );
//cv::waitKey(0);

//cv::imshow( "hdr2", hdr );
//cv::waitKey(0);


//cv::imshow( "Original", img_cv_rect_t );
//cv::waitKey(0);

float k_2[3][3] = {{1.0f/9.0f,1.0f/9.0f,1.0f/9.0f},{1.0f/9.0f,1.0f/9.0f,1.0f/9.0f},{1.0f/9.0f,1.0f/9.0f,1.0f/9.0f}};
float k_1[3][3] = {{-1.0f,-1.0f,-1.0f},{-1.0f,9.0f,-1.0f},{-1.0f,-1.0f,-1.0f}};
cv::Mat kernel(3, 3, CV_32F,k_1);
cv::Mat kernel2(3, 3, CV_32F,k_2);

float k_D[3][3] = {{1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f}};
cv::Mat kernel_dilate(3, 3, CV_32F,k_D);
float k_D4[4][4] = {{1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f}};
cv::Mat kernel_dilate4(4, 4, CV_32F,k_D4);
float k_D5[5][5] = {{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f}};
cv::Mat kernel_dilate5(5, 5, CV_32F,k_D5);


//float k_D5[5][5] = {{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f},{1.0f,1.0f,1.0f,1.0f,1.0f}};
//cv::Mat kernel_dilate5(4, 4, CV_32F,k_D4);


cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);
//cv::imshow( "K_1", img_cv_rect_t );
//cv::waitKey(10000);

cv::filter2D(img_cv_rect_t, img_cv_rect_t, -1, kernel);



//cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);

//cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);
//cv::filter2D(img_cv_rect_t, img_cv_rect_t, -1, kernel);
//cv::imshow( "K_2", img_cv_rect_t );
//cv::waitKey(10000);




cv::Mat gx, gy, img_cv_rect_t2,  angle;




cv::Sobel(img_cv_rect_t, gx, CV_32F, 1, 0, 1);
cv::Sobel(img_cv_rect_t, gy, CV_32F, 0, 1, 1);
cv::cartToPolar(gx, gy,img_cv_rect_t, angle);
//cv::imshow( "polar", img_cv_rect_t );
//cv::waitKey(1000);

//cv::imshow( "angle", angle );
//cv::waitKey(1000);

/**trial**/
//cv::threshold(img_cv_rect_t, img_cv_rect_t2, 27, 255, cv::THRESH_TOZERO);
//cv::threshold(img_cv_rect_t, img_cv_rect_t2, 15, 255, cv::THRESH_TOZERO);
/**trial**/


//cv::imshow( "polar thresh", img_cv_rect_t2 );
//cv::waitKey(1000);




/**trial**/
cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);

//cv::filter2D(img_cv_rect_t, img_cv_rect_t, -1, kernel);
//cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);
//cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel2);
//cv::filter2D(img_cv_rect_t, img_cv_rect_t,-1, kernel);

//img_cv_rect_t2 = img_cv_rect_t;
//cv::morphologyEx(img_cv_rect_t2,img_cv_rect_t2, cv::MORPH_DILATE, kernel_dilate);
//img_cv_rect_t2.assignTo(img_cv_rect_t2,CV_8UC1);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t2, 0, 255, cv::THRESH_OTSU);
//cv::imshow( "tresh 1", img_cv_rect_t2 );
//cv::waitKey(1000);



//img_cv_rect_t.assignTo(img_cv_rect_t,CV_8UC1);
//cv::threshold(img_cv_rect_t, img_cv_rect_t, 0, 255, cv::THRESH_OTSU);
/**trial**/




//cv::morphologyEx(img_cv_rect_t2,img_cv_rect_t2, cv::MORPH_OPEN, kernel_dilate4);

//cv::imshow( "Morph open", img_cv_rect_t2 );
//cv::waitKey(1000);

//cv::morphologyEx(img_cv_rect_t2,img_cv_rect_t2, cv::MORPH_CLOSE, kernel_dilate5);

//cv::imshow( "Morph close", img_cv_rect_t2 );
//cv::waitKey(1000);


//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t2, 7, 255, cv::THRESH_TOZERO);

//cv::imshow( "tresh 2", img_cv_rect_t2 );
//cv::waitKey(1000);





//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t2, 10, 255, cv::THRESH_TOZERO);

//cv::imshow( "tresh 3", img_cv_rect_t2 );
//cv::waitKey(1000);


//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t2, 10, 255, cv::THRESH_TOZERO);

//cv::imshow( "tresh 4", img_cv_rect_t2 );
//cv::waitKey(1000);


//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t2, 10, 255, cv::THRESH_TOZERO);

//cv::imshow( "tresh 5", img_cv_rect_t2 );
//cv::waitKey(1000);


//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t, 10, 255, cv::THRESH_TOZERO);

//cv::imshow( "tresh 6", img_cv_rect_t2 );
//cv::waitKey(1000);


//cv::filter2D(img_cv_rect_t2, img_cv_rect_t2,-1, kernel2);
//cv::threshold(img_cv_rect_t2, img_cv_rect_t, 5, 255, cv::THRESH_BINARY);

//cv::imshow( "tresh 5", img_cv_rect_t );
//cv::waitKey(1000);



//cv::dilate(img_cv_rect_t,img_cv_rect_t2,kernel_dilate);
////cv::imshow( "Dilate", img_cv_rect_t2 );
////cv::waitKey(10000);


////cv::morphologyEx(img_cv_rect_t,img_cv_rect_t2, cv::MORPH_OPEN, kernel_dilate);
//cv::morphologyEx(img_cv_rect_t2,img_cv_rect_t, cv::MORPH_CLOSE, kernel_dilate);
////cv::imshow( "Morph1", img_cv_rect_t );
////cv::waitKey(1000);
//cv::morphologyEx(img_cv_rect_t,img_cv_rect_t2, cv::MORPH_OPEN, kernel_dilate4);
////cv::imshow( "Morph3", img_cv_rect_t2 );
////cv::waitKey(1000);

//cv::morphologyEx(img_cv_rect_t2,img_cv_rect_t, cv::MORPH_CLOSE, kernel_dilate4);

//cv::erode(img_cv_rect_t,img_cv_rect_t,kernel_dilate);
//cv::imshow( "erode", img_cv_rect_t );
//cv::waitKey(1000);


//cv::imshow( "Morph2", img_cv_rect_t );
//cv::waitKey(1000);


camera__[itmParameters][itmCapture][itmFrontLight] = false;
camera__[itmParameters][itmCapture][itmProjector] = true;
camera__[itmParameters][itmCapture][itmAutoExposure] = true;
NxLibCommand (cmdEstimateDisparitySettings).execute();

try {
camera__[itmParameters][itmCapture][itmHdr] = false;
} catch (NxLibException& ex) {
	ensensoExceptionHandling(ex,"de33");
}
camera__[itmParameters][itmCapture][itmGain] = 1;
camera__[itmParameters][itmCapture][itmGainBoost] = false;
camera__[itmParameters][itmCapture][itmPixelClock] = 43;
camera__[itmParameters][itmCapture][itmMultiExposureFactor] = 2;
{
	NxLibCommand cam(cmdCapture);
	cam.parameters()[itmTimeout] = 25000;
	cam.execute();

	NxLibCommand (cmdComputeDisparityMap).execute();
	NxLibCommand (cmdComputePointMap).execute();


//	cam.execute();

//	cam.execute();

}
NxLibCommand (cmdCapture).execute();
NxLibCommand (cmdComputeDisparityMap).execute();
NxLibCommand (cmdComputePointMap).execute();

if (capture_params__.flex_view > 1)
{

	camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);


}

//camera__[itmParameters][itmCapture][itmProjector] = true;
//camera__[itmParameters][itmCapture][itmAutoExposure] = true;
//camera__[itmParameters][itmCapture][itmExposure] = 3.0;
{
	NxLibCommand cam(cmdCapture);
	cam.parameters()[itmTimeout] = 25000;
	cam.execute();

}
//NxLibCommand (cmdCapture).execute();
NxLibCommand (cmdComputeDisparityMap).execute();
NxLibCommand (cmdComputePointMap).execute();
	//Get 2D image raw data
	raw_img_r.clear();
	raw_img_l.clear();
	int photos_set = capture_params__.flex_view;
	photos_set = photos_set - (photos_set % 4);

		camera__[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);
		camera__[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points, 0);
/*
	raw_img.resize((unsigned int)ww*(unsigned int)hh);

	for (size_t i = 0; i < raw_img.size(); i++)
	{
		raw_img[i] = 0;

		for (size_t j = 0; j < raw_img_l.size(); j++)
		{

				raw_img[i] = raw_img[i]  + raw_img_l[j][i];



		}
		raw_img[i] = raw_img[i]/raw_img_l.size();

	}*/



//	if (aaaa > 1){
		//cv::imshow( "Raw", img_cv_raw_now );
//		cv::imshow( "Rect", img_cv_rect_now );
//		cv::waitKey(0);
//	}
//		aaaa++;

		//camera__[itmParameters][itmCapture][itmProjector] = true;

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
				_p_cloud.points.at(kk).intensity = static_cast<int>(img_cv_rect_t.at<float>(ii,jj));//raw_img[(ii*_p_cloud.width + jj)];//raw_img[kk];
				//std::cout << static_cast<int>(img_cv_rect_t.at<uchar>(ii*_p_cloud.width + jj)) << std::endl;
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
					_p_cloud.points.at(kk).intensity = static_cast<int>(img_cv_rect_t.at<uchar>(ii*_p_cloud.width + jj));
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
//	_p_cloud.is_dense = capture_params__.dense_cloud;

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

	std::cout << "Configuring capture " <<  std::endl;
	camera__[itmParameters][itmCapture][itmAutoExposure] = capture_params__.auto_exposure;

	camera__[itmParameters][itmCapture][itmAutoGain] = capture_params__.auto_exposure;
	camera__[itmParameters][itmCapture][itmExposure    ] = static_cast<double>(capture_params__.exposure_time); //TODO check if requires cast to double.
	//camera__[itmParameters][itmCapture][itmHdr] = true;
	camera__[itmParameters][itmCapture][itmMode] = "Rectified";

//	camera__[itmParameters][itmDisparityMap][itmPostProcessing][itmFilling][itmBorderSpread] = 5;
//	camera__[itmParameters][itmDisparityMap][itmPostProcessing][itmMedianFilterRadius] = 3;
//	camera__[itmParameters][itmDisparityMap][itmPostProcessing][itmSpeckleRemoval][itmComponentThreshold] = 5;
//	camera__[itmParameters][itmCapture][itmMode] = 'Raw';

//camera__[itmParameters][itmCapture][itmProjector] = false;




	//camera__[itmParameters][itmCapture][itmProjector] = false;


	if (capture_params__.flex_view < 2)
	{

		flexview_enabled__ = false;
		camera__[itmParameters][itmCapture][itmFlexView] = flexview_enabled__;

	}
	else
	{

		flexview_enabled__ = true;
		camera__[itmParameters][itmCapture][itmFlexView] = static_cast<int>(capture_params__.flex_view);

		camera__[itmParameters][itmCapture][itmGainBoost] = true;

	}

	if (capture_params__.flex_view > 8){
			camera__[itmParameters][itmDisparityMap][itmStereoMatching][itmMethod] = "Correlation";
		camera__[itmParameters][itmDisparityMap][itmStereoMatching][itmWindowRadius] = 1;
	}

	std::cout << "finished configuring capture " <<  std::endl;


}



int Device::HandsEyeCalibration(const ensenso_nx::HECalibrationGoalConstPtr &__goal, const ensenso_nx::HECalibrationResultConstPtr &__result)
{

	//MISSING CHECK IF A PATTERN IS OBTAINED OR NOT. IN CASE IT IS NOT OBTAINED, REPEAT.

	// will need to adapt this line to the size of the calibration pattern that you are using.
	// Discard any pattern observations that might already be in the pattern buffer.
	std::cout << "deb1: " << std::endl;
		int ww, hh;
	NxLibCommand(cmdDiscardPatterns).execute();
	std::cout << "deb2: " << std::endl;

	NxLibCommand decode(cmdCollectPattern);
	std::cout << "deb3: " << std::endl;

	motion_server::FreeModeGoal goal_free_mode;
	std::cout << "deb4: " << std::endl;

	goal_free_mode.until_button_pressed.data = true;
	std::cout << "deb5: " << std::endl;

	int nx_return_code;
	std::cout << "deb6: " << std::endl;

	double timestamp;
	std::cout << "deb7: " << std::endl;

	if (!he_cal_params__.decode_data)
	{
		std::cout << "deb71: " << std::endl;
			camera__[itmParameters][itmPattern][itmGridSpacing] = he_cal_params__.grid_spacing;
	}
	else
	{
		/*
		std::cout << "deb72: " << std::endl;
			decode.parameters()[itmDecodeData] = true;

			try {
				decode.execute();
			} catch (NxLibException& ex)
			{
				ensensoExceptionHandling(ex,"decode_data");
			}
*/
	}



	// Turn off the camera's projector so that we can observe the calibration pattern.
	std::cout << "deb73: " << std::endl;
	camera__[itmParameters][itmCapture][itmProjector] = false;
	std::cout << "deb74: " << std::endl;
	camera__[itmParameters][itmCapture][itmFrontLight] = true;

	// You can adapt this number depending on how accurate you need the
	// calibration to be.
	for (int i = 0; i < __goal->position_number; i++) {
			// Move your robot to a new position from which the pattern can be seen. It might be a good idea to
		std::cout << "deb8: " << std::endl;

			free_mode__.sendGoal(goal_free_mode);
			std::cout << "Waiting to confirm new position" << std::endl;
			free_mode__.waitForResult();
			std::cout << "New position have been declared" << std::endl;

			//Make sure that the robot is not moving anymore. You might want to wait for a few seconds to avoid
			//any oscillations.
			sleep(2);

			// Observe the calibration pattern and store the observation in the pattern buffer.
			try{
			NxLibCommand capture(cmdCapture);
			capture.parameters()[itmTimeout] = 25000;
			capture.parameters()[itmCameras] = device_params__.serial_num;
			capture.execute();
			} catch (NxLibException& ex)
			{
				ensensoExceptionHandling(ex,"capture");


			}
			bool foundPattern = false;
			try {
				std::vector<std::vector<double>> matrix_t;
				matrix_t.resize(3);
				matrix_t[0].resize(3);
				matrix_t[1].resize(3);
				matrix_t[2].resize(3);
					NxLibCommand collectPattern(cmdCollectPattern);
					collectPattern.parameters()[itmCameras] = device_params__.serial_num;
					collectPattern.execute();

					NxLibItem item_pattern = collectPattern.result()[itmStereo][0][itmLeft][itmTransformation];
					//std::cout << item_pattern.asDouble() << std::endl;
					collectPattern.result()[itmStereo][0][itmLeft][itmTransformation].getBinaryData(&nx_return_code,matrix_t,&timestamp);

					std::cout << "Matrix: " << std::endl;
					for (size_t i = 0; i< matrix_t.size(); i++)
					{
						for (size_t j = 0; j< matrix_t[i].size(); j++)
						{
							//std::cout << item_pattern[i][j].asDouble() << " ";
							std::cout << matrix_t[i][j] << " ";
						}
						std::cout << std::endl;
					}
					foundPattern = true;
			} catch (NxLibException& ex)
			{
				ensensoExceptionHandling(ex,"collect_pattern");


			}

			if (foundPattern) {
					// We actually found a pattern. Get the current pose of your robot (from which the pattern was
					// observed) and store it somewhere.
					//geometry_msgs::TransformStamped table_to_ensenso = tf2_buffer__.lookupTransform("table", "world", ros::Time(0), ros::Duration(0.5));
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
