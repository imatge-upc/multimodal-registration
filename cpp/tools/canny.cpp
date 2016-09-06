#include <iostream>

#include <limits>
#include <vector>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include <SimppleLogger.hpp>


int
main (int argc, char* argv[])
{
	std::string path_in, path_out;
	float color_noise;
    bool blur;
	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input,i", boost::program_options::value<std::string>(&path_in)->required())
		("noise", boost::program_options::value<float>(&color_noise)->default_value(0.0))
        ("blur", boost::program_options::value<bool>(&blur)->default_value(false))
        ("output", boost::program_options::value<std::string>(&path_out)->required())
	;

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	//std::string imagepath = "/opt/Datasets/recording-oni/images/IMG_20151125_151247.jpg";
//
//	// Working with image
    int lowThreshold=100;
    int ratio = 3;
    int kernel_size = 3;
    // Reading camera calibration parameters
    cv::Mat src_full, src_cropped, src_bg, src_noise, im_noise, detected_edges, dst;
    cv::Rect crop; crop.x = 0; crop.y = 0; crop.width = 128; crop.height = 128;
    src_full = cv::imread(path_in, 1);
    src_cropped = cv::Mat(src_full, crop);
    cv::cvtColor( src_cropped, src_bg, CV_BGR2GRAY);
      for(int i=0;i<src_bg.rows;i++)
         {
            for (int j = 0; j < src_bg.cols; ++j)
            {
                int d_c = 0;
                if (color_noise>0) d_c = rand()*2*color_noise/RAND_MAX;
                src_bg.at<uchar>(j,i) = std::max(0, std::min(255, int(src_bg.at<uchar>(j,i)) + d_c));
            }
         }
    if (blur){
        cv::blur(src_bg, detected_edges, cv::Size(3,3) );
    } else {
        detected_edges = src_bg;
    }
    cv::Canny( detected_edges, dst, lowThreshold, lowThreshold*ratio, kernel_size );

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

     for(int i=0;i<src_bg.rows;i++)
     {
        uchar* p; p = dst.ptr<uchar>(i); // point to first pixel in row
    	for (int j = 0; j < src_bg.cols; ++j)
    	{
    		pcl::PointXYZI point;
    		point.x = float(i)/16.0-8.0;
    		point.y = -float(j)/16.0+8.0;
    		point.z = 0.027;
    		point.intensity = int(p[j])/255;
    		point_cloud_ptr->points.push_back(point);
    	}

     }
     point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
     point_cloud_ptr->height = 1;
     pcl::io::savePLYFileASCII (path_out, *point_cloud_ptr);
    return (0);
}
