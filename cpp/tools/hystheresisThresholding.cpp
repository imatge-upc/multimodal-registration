#include <iostream>
#include <fstream>
#include <typeinfo>

#include <limits>
#include <vector>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/filter_indices.h>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <edges.h>

#include <cstdlib>

#include <chrono>
#include <SimppleLogger.hpp>


typedef pcl::PointXYZI PointTypeIn;


void setViewerCamera( pcl::visualization::PCLVisualizer &viewer, std::string cam_params_str){
	if(cam_params_str.compare("")!=0){
		pcl::visualization::Camera cam;
		std::stringstream str(cam_params_str);
		std::vector<double> cam_params((std::istream_iterator<double>(str)),std::istream_iterator<double>());
		cam.focal[0] = cam_params[0];
		cam.focal[1] = cam_params[1];
		cam.focal[2] = cam_params[2];
		cam.pos[0] = cam_params[3];
		cam.pos[1] = cam_params[4];
		cam.pos[2] = cam_params[5];
		cam.view[0] = cam_params[6];
		cam.view[1] = cam_params[7];
		cam.view[2] = cam_params[8];
		cam.clip[0] = cam_params[9];
		cam.clip[1] = cam_params[10];
		cam.fovy = cam_params[11];
		cam.window_size[0] = cam_params[12];
		cam.window_size[1] = cam_params[13];
		cam.window_pos[0] = cam_params[14];
		cam.window_pos[1] = cam_params[15];
		viewer.setCameraParameters(cam, 0);
	}
}


int
main (int argc, char* argv[])
{
    pplelog::setLogLevel("utils/edges", pplelog::DEBUG);

	std::string path_in, path_out_pcd, path_out_ply, path_out_orig, path_out, path_out_txt;
	bool x, show;
	float low_thresh, high_thresh;
	std::string cam_params;

	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input,i", boost::program_options::value<std::string>(&path_in)->required(),
				"Input point cloud in PCD format")
		("low_thresh", boost::program_options::value<float>(&low_thresh)->default_value(10.0))
		("high_thresh", boost::program_options::value<float>(&high_thresh)->default_value(100.0))
		("output-ply", boost::program_options::value<std::string>(&path_out_ply)->default_value(""))
		("x", boost::program_options::value<bool>(&x)->default_value(false),
				"Show graphic data (unset when working with no X)")
		("camera", boost::program_options::value<std::string>(&cam_params)->default_value(""),
				"Vector with the 16 camera parameters:  'focal[3], pos[3], view[3], clip[2], fovy, window_size[2], window_pos[2]'")
		;
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);

	pcl::PointCloud<PointTypeIn>::Ptr cloud(new pcl::PointCloud<PointTypeIn>);
    if (path_in.substr( path_in.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in);
            return (-1);
        }
    }
    else if (path_in.substr( path_in.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path_in, *cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in);
            return (-1);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path_in);
        return (-1);
    }
    std::cout << "Loaded " << cloud->width << " * " << cloud->height
              << " data points from " << path_in << std::endl;

	pcl::PointCloud<PointTypeIn>::Ptr cloud_out(new pcl::PointCloud<PointTypeIn>);

	std::vector<bool> cloud_toVisit(cloud->points.size(), false);
    std::vector<bool> cloud_visited(cloud->points.size(), false);

	for (int j = 0; j < cloud->points.size(); j++) {
			if(cloud->points[j].intensity > high_thresh){
				cloud_toVisit[j]=true;
		}
	}
	// ToDo: cloud_threshold to know points above threshold;
	int K=10;
	pcl::KdTreeFLANN<PointTypeIn> kdtree;
    std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	kdtree.setInputCloud(cloud);
	auto it = std::find(cloud_toVisit.begin(), cloud_toVisit.end(),true);
	while (it < cloud_toVisit.end()){ // Search all lines on image
		int elem = std::distance(cloud_toVisit.begin(), it);
		cloud_visited[elem] = true;
		cloud_toVisit[elem] = false;
		if(cloud->points[elem].intensity > low_thresh){
			kdtree.nearestKSearch (cloud->points[elem], K, pointIdxNKNSearch, pointNKNSquaredDistance);
			for (size_t i=0; i<pointIdxNKNSearch.size(); i++){
				if (!cloud_visited[pointIdxNKNSearch[i]] && !cloud_toVisit[pointIdxNKNSearch[i]]){
					cloud_toVisit[pointIdxNKNSearch[i]] = true;
					break;
				}
			}
			cloud_out->push_back(cloud->points[elem]);
		}
		it = std::find(cloud_toVisit.begin(), cloud_toVisit.end(),true);
	}

    if (path_out_ply.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_out_ply, *cloud_out);
    }

}

