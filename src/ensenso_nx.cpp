#include "ensenso_nx.h"

namespace EnsensoNx
{

Device::Device(const DeviceParams & _params)
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
    std::string serial_num = camera_[itmSerialNumber].asString();
    
    //open camera
    NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
    open.parameters()[itmCameras] = serial_num; // Set parameters for the open command
    open.execute();
    std::cout << "EnsensoNx::Device: Camera open. SN: " << serial_num << std::endl; 

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

void Device::capture()
{
    
}


}//close namespace