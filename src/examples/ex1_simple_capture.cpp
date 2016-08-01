//open the Ensenso camera, get one snapshot, visualize it, and wait for "q" to close

//PCL visualiser
#include <pcl/visualization/pcl_visualizer.h>

//EnsensoNx Lib
#include "../ensenso_nx.h"

//std
#include <iostream>

//main
int main(int argc, char **argv)
{
    //ensenso camera
    EnsensoNx::Device camera;
    
    //configure camera (auto exposure and exposure time)
    EnsensoNx::CaptureParams capture_params;
    capture_params.auto_exposure_ = false; 
    capture_params.exposure_time_ = 10; //ms
    camera.configureCapture(capture_params);
    
    //point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    
    //visualization window
    pcl::visualization::PCLVisualizer viewer_("EnsensoNx Snapshot");
    
    //get a single snapshot in this thread
    camera.capture(*p_cloud_);
    std::cout << "Cloud captured! " << std::endl; 
    std::cout << "\twidth: " << p_cloud_->width << std::endl;
    std::cout << "\theight: " << p_cloud_->height << std::endl; 
    
    //visualization starts here
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> p_cloud_color_handler(p_cloud_, 255, 255, 255);
    viewer_.addPointCloud (p_cloud_, p_cloud_color_handler, "snapshot");
    viewer_.addCoordinateSystem (1.0, 0);
    viewer_.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "snapshot");
    viewer_.setPosition(300,200); // Setting visualiser window position

    // Display the visualiser until 'q' key is pressed
    while (!viewer_.wasStopped ()) { 
        viewer_.spinOnce ();
    }    
    
    //bye
    return 1;
}
