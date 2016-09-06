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
	std::string path_in, path_out, path_out_ply, dist_coeffs, cx_cy_fx_fy;
	bool show_result;
	int low_thresh, high_thresh;
	float scale;
	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input,i", boost::program_options::value<std::string>(&path_in)->required())
		("scale", boost::program_options::value<float>(&scale)->default_value(1.0))
        ("dist_coeffs", boost::program_options::value<std::string>(&dist_coeffs)->required())
        ("cx_cy_fx_fy", boost::program_options::value<std::string>(&cx_cy_fx_fy)->required())
		("output-ply", boost::program_options::value<std::string>(&path_out_ply)->default_value(""))
	;

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	//std::string imagepath = "/opt/Datasets/recording-oni/images/IMG_20151125_151247.jpg";
//
//	// Working with image

    // Reading camera calibration parameters
    std::stringstream str_1(dist_coeffs);
    std::vector<float> dist_coeffs_vec((std::istream_iterator<double>(str_1)), std::istream_iterator<double>());
    std::stringstream str_2(cx_cy_fx_fy);
    std::vector<float> cx_cy_fx_fy_vec((std::istream_iterator<double>(str_2)), std::istream_iterator<double>());

    cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << cx_cy_fx_fy_vec[2], 0, cx_cy_fx_fy_vec[0], 
                                 0, cx_cy_fx_fy_vec[3], cx_cy_fx_fy_vec[1],
                                 0, 0, 1);
    std::cout << "created matrix" << std::endl;

    cv::Mat src_full, src_und, dst;

    src_full = cv::imread(path_in, 1);
    // cv::undistort(src_full, src_und, cameraMatrix, dist_coeffs_vec);
    src_und = src_full;
    cv::resize(src_und, dst, cv::Size(src_full.cols*scale, src_full.rows*scale));

    if (path_out_ply.compare("")!=0){
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

		 for(int i=0;i<dst.rows;i++)
		 {
			cv::Vec3b* pixel = dst.ptr<cv::Vec3b>(i); // point to first pixel in row
			for (int j = 0; j < dst.cols; ++j)
			{
				pcl::PointXYZRGB point;
				point.y = (-i+cx_cy_fx_fy_vec[0])/cx_cy_fx_fy_vec[2];
				point.x = (j-cx_cy_fx_fy_vec[1])/cx_cy_fx_fy_vec[3];
				point.z = 1;
				point.r = int(pixel[j][2]);
				point.g = int(pixel[j][1]);
				point.b = int(pixel[j][0]);
				point_cloud_ptr->points.push_back(point);
			}

		 }
		 point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
		 point_cloud_ptr->height = 1;
		 pcl::io::savePLYFileASCII (path_out_ply, *point_cloud_ptr);
    }
    return (0);
}
