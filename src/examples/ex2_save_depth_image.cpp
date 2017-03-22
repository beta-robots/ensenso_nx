//open the Ensenso camera, get one snapshot, and savethe depth image

//Open CV save as
#include "opencv2/highgui.hpp"

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    //opencv image and write params
    cv::Mat image; 
    std::vector<int> image_write_params;
    
    //get a single snapshot in this thread
    camera.capture(*p_cloud, image);
    std::cout << "Cloud & Image captured! " << std::endl; 
    std::cout << "\tCloud width: " << p_cloud->width << std::endl;
    std::cout << "\tCloud height: " << p_cloud->height << std::endl; 
    std::cout << "\tImage width: " << image.cols<< std::endl;
    std::cout << "\tImage height: " << image.rows << std::endl; 
    
    //Save image at Desktop
    image_write_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    image_write_params.push_back(9);
    cv::imwrite("/home/andreu/Desktop/image1.png", image, image_write_params);
    cv::imwrite("/home/andreu/Desktop/image1.pgm", image);
    
    //bye
    return 1;
}
