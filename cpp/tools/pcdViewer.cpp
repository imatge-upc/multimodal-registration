/*
 * pcdViewer.cpp
 *
 *  Created on: Dec 4, 2015
 *      Author: alba
 */

#include <iostream>

#include <limits>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "boost/filesystem.hpp"

#include <pcl/visualization/cloud_viewer.h>
#include <boost/program_options.hpp>


typedef pcl::PointXYZRGB PointTypeIn;
typedef pcl::PointXYZI PointTypeOut;

std::vector<std::string> paths;
int i;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
		void* viewer_void) {
	bool drawCloud = false;
	if (event.getKeySym() == "n" && event.keyDown()) {
		std::cout << "n was pressed => going to next image" << std::endl;
		i += 1;
		if (i >= paths.size()) {
			i = 0;
		} else if (i < 0) {
			i = paths.size() - 1;
		}
		drawCloud = true;
	} else if (event.getKeySym() == "p" && event.keyDown()) {
		std::cout << "p was pressed => going to previous image" << std::endl;
		i -= 1;
		if (i >= paths.size()) {
			i = 0;
		} else if (i < 0) {
			i = paths.size() - 1;
		}
		drawCloud = true;
	}
	if (drawCloud) {
		pcl::PointCloud<PointTypeIn>::Ptr cloud(
				new pcl::PointCloud<PointTypeIn>);
		if (pcl::io::loadPCDFile<PointTypeIn>(paths[i], *cloud) == -1) {
			PCL_ERROR("Couldn't read the file %s \n", paths[i]);
		} else {
			std::cout << "Loaded " << paths[i] << std::endl;
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
					*static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
			viewer->removePointCloud();
			viewer->addPointCloud(cloud);
		}
	}

}



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
	std::string path;
	std::string cam_params;
	i = -1;
	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("path,p", boost::program_options::value<std::string>(&path)->required(),
				"Input point cloud in PCD format")
		("camera", boost::program_options::value<std::string>(&cam_params)->default_value(""),
				"Vector with the 16 camera parameters:  'focal[3], pos[3], view[3], clip[2], fovy, window_size[2], window_pos[2]'")
		;
	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);




	boost::filesystem::recursive_directory_iterator iter(path), eod;

	BOOST_FOREACH(boost::filesystem::path const& k, std::make_pair(iter, eod)){
	    if (boost::filesystem::is_regular_file(k)){
	        std::cout << k.string() << std::endl;
	        paths.push_back(k.string());
	    }
	}
	std::sort(paths.begin(), paths.end());
	// Working with point clouds
	// Define initial cloud and keypoints
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	pcl::visualization::Camera cam;
	setViewerCamera(*viewer, cam_params);
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}




//    pcl::visualization::CloudViewer viewer1 ("All cloud");
//    viewer1.showCloud (cloud);
//    while (!viewer1.wasStopped ()){}

    return (0);
}



