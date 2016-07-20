#include "ensenso_nx.h"

namespace EnsensoNx
{

Device::Device()
{
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
    NxLibCommand (cmdClose).execute();
    std::cout << "EnsensoNx::Device: Camera closed." << std::endl; 
    
    //finalizes nx library
    nxLibFinalize();
}

void Device::configureCapture(const CaptureParams & _params)
{
    //sets capture configuration to the camera
    camera_[itmParameters][itmCapture][itmAutoExposure] = _params.auto_exposure_;
    camera_[itmParameters][itmCapture][itmExposure    ] = _params.exposure_time_;
    std::cout << "EnsensoNx::Device: Capture params set to:" << std::endl; 
    _params.print(); 
}

void Device::capture(pcl::PointCloud<pcl::PointXYZ> & _p_cloud)
{
    int ww, hh;
    std::vector<float> raw_points;
    
    // Capture images
    NxLibCommand (cmdCapture).execute();
    
    // Compute Disparity Map
    NxLibCommand (cmdComputeDisparityMap).execute();
    
    // Compute Point Cloud
    NxLibCommand (cmdComputePointMap).execute();

    // Get image dimensions
    camera_[itmImages][itmPointMap].getBinaryDataInfo(&ww, &hh, 0,0,0,0);
    
    //Get 3D image raw data 
    camera_[itmImages][itmPointMap].getBinaryData(raw_points, 0);
 
    //Move raw data to point cloud
    _p_cloud.width = (unsigned int)ww;
    _p_cloud.height = (unsigned int)hh;
    _p_cloud.resize(_p_cloud.width*_p_cloud.height);
    for(unsigned int ii = 0; ii<_p_cloud.height; ii++ )
    {
        for(unsigned int jj = 0; jj<_p_cloud.width; jj++ )
        {
            _p_cloud.points.at(ii*_p_cloud.width + jj).x = raw_points[(ii*_p_cloud.width + jj)*3];
            _p_cloud.points.at(ii*_p_cloud.width + jj).y = raw_points[(ii*_p_cloud.width + jj)*3 + 1];
            _p_cloud.points.at(ii*_p_cloud.width + jj).z = raw_points[(ii*_p_cloud.width + jj)*3 + 2];
        }
    }
    
}

}//close namespace
