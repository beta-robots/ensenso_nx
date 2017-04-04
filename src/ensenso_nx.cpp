#include "ensenso_nx.h"

namespace EnsensoNx
{

Device::Device()
{
    std::cout << "EnsensoNx::Device: Opening camera ..." << std::endl;
    
    //init nx library
    nxLibInitialize(true);

    // Create an object referencing the camera's tree item, for easier access:
    camera_ = nx_lib_root_[itmCameras][itmBySerialNo][0];
    if (!camera_.exists() || (camera_[itmType] != valStereo)) 
    {
        std::cout << "EnsensoNx::Device: Camera not found. Please connect a single stereo camera to your computer" << std::endl;
        return;
    }

    //get serial number of the connected camera
    device_params_.serial_num_ = camera_[itmSerialNumber].asString();
    
    //open camera
    NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
    open.parameters()[itmCameras] = device_params_.serial_num_; // Set parameters for the open command
    open.execute();
    std::cout << "EnsensoNx::Device: Camera open. SN: " << device_params_.serial_num_ << std::endl; 

}

Device::~Device()
{
    //close the camera
    std::cout << "EnsensoNx::Device: Closing camera ..." << std::endl; 
    NxLibCommand (cmdClose).execute();
    std::cout << "EnsensoNx::Device: Camera closed." << std::endl; 
    
    //finalizes nx library
    nxLibFinalize();
}

void Device::configureCapture(const CaptureParams & _params)
{
    //update class member
    capture_params_.auto_exposure_ = _params.auto_exposure_;
    capture_params_.exposure_time_ = _params.exposure_time_;
    capture_params_.dense_cloud_ = _params.dense_cloud_;
    
    //call protected member to set the configuration to the camera
    this->configureCapture(); 
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
    //std::vector<float> raw_points;
    int nx_return_code; 
    
    // Capture images
    NxLibCommand (cmdCapture).execute();
    
    // Compute Disparity Map
    NxLibCommand (cmdComputeDisparityMap).execute();
    
    // Compute Point Cloud
    NxLibCommand (cmdComputePointMap).execute();

    // Get image dimensions
    camera_[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);
    
    //Get 3D image raw data 
    camera_[itmImages][itmPointMap].getBinaryData(&nx_return_code, raw_points_, 0);
     
    //Move raw data to point cloud
    _p_cloud.width = (unsigned int)ww;
    _p_cloud.height = (unsigned int)hh;
    _p_cloud.resize(_p_cloud.width*_p_cloud.height);
    unsigned int kk = 0; 
    for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
    {
        for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
        {
            px = raw_points_[(ii*_p_cloud.width + jj)*3]; 
            if ( !std::isnan(px) )
            {
                _p_cloud.points.at(kk).x = px/1000.;
                _p_cloud.points.at(kk).y = raw_points_[(ii*_p_cloud.width + jj)*3 + 1]/1000.;
                _p_cloud.points.at(kk).z = raw_points_[(ii*_p_cloud.width + jj)*3 + 2]/1000.;
                kk++; 
            }
            else //in case of nan, check dense_cloud_ to fill in the cloud or not 
            {
                if (capture_params_.dense_cloud_)
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
    
    //debug message
//     std::cout << "Cloud capture: " << std::endl <<
//                  "\treturn code: " << nx_return_code << std::endl <<
//                  "\tnum points: " << raw_points_.size()/3 << std::endl <<
//                  "\twidth: " << ww << std::endl <<
//                  "\theight: " << hh << std::endl <<
//                  "\tvalid_points: " << kk << std::endl; 
        
    //return success
    return 1; 
}

int Device::capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud, cv::Mat & _d_image)
{
    //local vars
    int ww, hh;
    
    //get point cloud. raw_points_ class member is also set
    int ret_value = this->capture(_p_cloud); 
    
    //from point cloud, initialize a depth image
    camera_[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0); // Get image dimensions
    _d_image.create(hh, ww, CV_16UC1);
    //_d_image.create(_p_cloud.height, _p_cloud.width, CV_8UC1);
    
    //debug message
//     std::cout << "_p_cloud.height: " << _p_cloud.height << std::endl
//               << "_p_cloud.width: " << _p_cloud.width << std::endl
//               << "_d_image.rows: " << _d_image.rows << std::endl
//               << "_d_image.cols: " << _d_image.cols << std::endl;
    
    //fill image
    float depth_f, px;
    unsigned short depth_us; 
    //unsigned short depth_uc;
    for (unsigned int ii=0; ii<_d_image.rows; ii++)
    {
        for (unsigned int jj=0; jj<_d_image.cols; jj++)
        {
            //check if value is valid 
            //depth_f = _p_cloud.points.at(ii*_p_cloud.width+jj).z;
            px = raw_points_.at((ii*_d_image.cols+jj)*3); 
            if ( !std::isnan(px) )
            {
                //get depth data
                depth_f = raw_points_.at((ii*_d_image.cols+jj)*3+2)/1000.; 
                
                //convert depth to unsigned short between 1m and 2m
                if (depth_f > MAX_DEPTH) depth_f = MAX_DEPTH;
                if (depth_f < MIN_DEPTH) depth_f = MIN_DEPTH; 
                depth_us = (unsigned short)((depth_f-MIN_DEPTH)*(65535.));
                //depth_uc = (unsigned char)((depth_f-1.)*(255.));
                
                //set value to image
                _d_image.at<unsigned short>(ii,jj) = depth_us;
                //_d_image.at<unsigned char>(ii,jj) = depth_uc;
            }
            else //in case of nan, put max depth in the image (max intensity level)
            {
                _d_image.at<unsigned short>(ii,jj) = 65535;
            }
        }
    }

    //return value from above capture call
    return ret_value; 
}

//PROTECTED METHODS

void Device::configureCapture()
{
    //sets capture configuration to the camera
    camera_[itmParameters][itmCapture][itmAutoExposure] = capture_params_.auto_exposure_;
    camera_[itmParameters][itmCapture][itmExposure    ] = (double)capture_params_.exposure_time_;//TODO check if requires cast to double. 
    
    //print out
    //std::cout << "EnsensoNx::Device: Capture params set to:" << std::endl; 
    //capture_params_.print();   
}

}//close namespace
