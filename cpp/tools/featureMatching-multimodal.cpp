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
#include <pcl/registration/icp-lines.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>

#include <pcl/filters/filter_indices.h>

#include <pcl/visualization/cloud_viewer.h>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <edges.h>

#include <cstdlib>

#include <chrono>
#include <SimppleLogger.hpp>

#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>


typedef pcl::PointXYZRGB PointTypeIn;

utils::LineCloud<PointTypeIn> linecloud;
int i=0;


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

	std::string path_in_cloud, path_in_cloud_full, path_in_image, path_fincloud_proj, path_finimg, path_fincloud ;
	bool x, show;
	int K, filter_by_size;
	double neighbour_threshold, line_threshold, compare_line_threshold, filter_by_disparity;
	float weight_high_thresh, weight_low_thresh;
	std::string cam_params;
    std::string rt;

	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("cloud", boost::program_options::value<std::string>(&path_in_cloud)->required(),
				"Input point cloud in PCD format")
        ("cloud_full", boost::program_options::value<std::string>(&path_in_cloud_full)->default_value(""),
                "Input point cloud in PCD format")
		("image", boost::program_options::value<std::string>(&path_in_image)->required(),
				"Input point cloud in PCD format")
		("final_image", boost::program_options::value<std::string>(&path_finimg)->default_value(""))
		("final_cloud", boost::program_options::value<std::string>(&path_fincloud)->default_value(""))
		("final_cloud_projected", boost::program_options::value<std::string>(&path_fincloud_proj)->default_value(""))
        ("rt", boost::program_options::value<std::string>(&rt)->required())
		("x", boost::program_options::value<bool>(&x)->default_value(false),
				"Show graphic data (unset when working with no X)")
		("camera", boost::program_options::value<std::string>(&cam_params)->default_value(""),
				"Vector with the 16 camera parameters:  'focal[3], pos[3], view[3], clip[2], fovy, window_size[2], window_pos[2]'")
		;
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);

	pcl::PointCloud<PointTypeIn>::Ptr cloud_cloud(new pcl::PointCloud<PointTypeIn>);
	if (path_in_cloud.substr( path_in_cloud.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in_cloud, *cloud_cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_cloud);
            return (-1);
        }
    }
    else if (path_in_cloud.substr( path_in_cloud.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path_in_cloud, *cloud_cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_cloud);
            return (-1);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path_in_cloud);
        return (-1);
    }
    std::cout << "Loaded " << cloud_cloud->width << " * " << cloud_cloud->height
              << " data points from " << path_in_cloud << std::endl;

	// PointTypeIn p1; p1.x = 0; p1.y=0; p1.z = 5;
	// cloud_cloud->push_back(p1);
	// PointTypeIn p2; p2.x = 2; p2.y=-3; p2.z = 4;
	// cloud_cloud->push_back(p2);
	// PointTypeIn p3; p3.x = -3; p3.y=2; p3.z = 6;
	// cloud_cloud->push_back(p3);
	// PointTypeIn p4; p4.x = 1; p4.y=-1; p4.z = 3;
	// cloud_cloud->push_back(p4);
	// PointTypeIn p5; p5.x = 2; p5.y=3; p5.z = 7;
	// cloud_cloud->push_back(p5);
	// PointTypeIn p6; p6.x = 3; p6.y=4; p6.z = 2;
	// cloud_cloud->push_back(p6);

	pcl::PointCloud<PointTypeIn>::Ptr cloud_image(new pcl::PointCloud<PointTypeIn>);
	if (path_in_image.substr( path_in_image.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path_in_image, *cloud_image) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_image);
            return (-1);
        }
    }
    else if (path_in_image.substr( path_in_image.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path_in_image, *cloud_image) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path_in_image);
            return (-1);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path_in_image);
        return (-1);
    }
    std::cout << "Loaded " << cloud_image->width << " * " << cloud_image->height
              << " data points from " << path_in_image << std::endl;	

    float focal = cloud_image->points[0].z;

    std::cout << "Focal: " << focal << std::endl;    

	// Eigen::Affine3f transform = Eigen::Affine3f::Identity();
 //  	transform.translation() << 0.1, 0.03, 0.2;
 //  	transform.rotate (Eigen::AngleAxisf (M_PI/4*0.1, Eigen::Vector3f::UnitX()));	
 //  	transform.rotate (Eigen::AngleAxisf (M_PI/7*0.1, Eigen::Vector3f::UnitY()));	
 //  	transform.rotate (Eigen::AngleAxisf (M_PI/5*0.1, Eigen::Vector3f::UnitZ()));	

 //  	pcl::PointCloud<PointTypeIn>::Ptr transformed_cloud (new pcl::PointCloud<PointTypeIn> ());
	// pcl::transformPointCloud (*cloud_cloud, *transformed_cloud, transform);

	// for (i=0; i<transformed_cloud->points.size(); i++){
	// 	  PointTypeIn p;
	//       p.x = transformed_cloud->points[i].x/transformed_cloud->points[i].z;
	//       p.y = transformed_cloud->points[i].y/transformed_cloud->points[i].z;
	//       p.z = 1;
	//       cloud_image->points.push_back(p);
	// }

	// pcl::PointCloud<PointTypeIn>::Ptr cloud_image2(new pcl::PointCloud<PointTypeIn>);
	// for (i=0; i<cloud_cloud->points.size(); i++){
	// 	  PointTypeIn p;
	//       p.x = cloud_cloud->points[i].x*focal/cloud_cloud->points[i].z;
	//       p.y = cloud_cloud->points[i].y*focal/cloud_cloud->points[i].z;
	//       p.z = focal;
	//       cloud_image2->points.push_back(p);
	// }
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    std::stringstream str(rt);
    std::vector<float> rt_v((std::istream_iterator<float>(str)),std::istream_iterator<float>());
    transform.translation() << rt_v[0], rt_v[1], rt_v[2];
    std::cout << "RT: " <<rt_v[0] << " " << rt_v[1] << " " << rt_v[2] << std::endl;
    transform.rotate (Eigen::AngleAxisf (rt_v[3],Eigen::Vector3f::UnitX()));    
    transform.rotate (Eigen::AngleAxisf (rt_v[4],Eigen::Vector3f::UnitY()));
    transform.rotate (Eigen::AngleAxisf (rt_v[5],Eigen::Vector3f::UnitZ()));    

	pcl::IterativeClosestPointLine<PointTypeIn, PointTypeIn> icp;
	icp.setMaximumIterations(10000);
	icp.setInputSource(cloud_image);
	icp.setInputTarget(cloud_cloud);
	pcl::PointCloud<PointTypeIn> image_out;
	icp.align(image_out, transform.matrix());
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	// std::cout << transform.matrix().inverse() << std::endl;

	pcl::PointCloud<PointTypeIn> cloud_out, cloud_projected_out;
    pcl::PointCloud<PointTypeIn> cloud_out_inv, cloud_projected_out_inv;

	pcl::transformPointCloud (*cloud_cloud, cloud_out, (icp.getFinalTransformation()).inverse().eval());

	for (i=0; i<cloud_out.points.size(); i++){
		  PointTypeIn p;
	      p.x = cloud_out.points[i].x*focal/cloud_out.points[i].z;
	      p.y = cloud_out.points[i].y*focal/cloud_out.points[i].z;
	      p.z = focal;
	      cloud_projected_out.points.push_back(p);
	}

    if (path_in_cloud_full.compare("") != 0) {
        pcl::PointCloud<PointTypeIn>::Ptr cloud_image2(new pcl::PointCloud<PointTypeIn>);
        if (path_in_cloud_full.substr( path_in_cloud_full.length() - 3 ) == "pcd")
        {
            if (pcl::io::loadPCDFile<PointTypeIn>(path_in_cloud_full, *cloud_image2) == -1) {
                PCL_ERROR("Couldn't read the file %s \n", path_in_cloud_full);
                return (-1);
            }
        }
        else if (path_in_cloud_full.substr( path_in_cloud_full.length() - 3 ) == "ply")
        {
            if (pcl::io::loadPLYFile<PointTypeIn>(path_in_cloud_full, *cloud_image2) == -1) {
                PCL_ERROR("Couldn't read the file %s \n", path_in_cloud_full);
                return (-1);
            }
        } else {
            PCL_ERROR("File extension for file %s not recognized\n", path_in_cloud_full);
            return (-1);
        }
        std::cout << "Loaded " << cloud_image2->width << " * " << cloud_image2->height
                  << " data points from " << path_in_cloud_full << std::endl;    
        pcl::transformPointCloud (*cloud_image2, *cloud_image2, (icp.getFinalTransformation()).inverse().eval());
        pcl::io::savePLYFileASCII ("miiine.ply", *cloud_image2);
    }




	pcl::transformPointCloud (*cloud_cloud, cloud_out_inv, icp.getFinalTransformation());

	for (i=0; i<cloud_out_inv.points.size(); i++){
		  PointTypeIn p;
	      p.x = cloud_out_inv.points[i].x*focal/cloud_out_inv.points[i].z;
	      p.y = cloud_out_inv.points[i].y*focal/cloud_out_inv.points[i].z;
	      p.z = focal;
	      cloud_projected_out_inv.points.push_back(p);
	}

    if (path_finimg.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_finimg, image_out);
    }
    if (path_fincloud.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_fincloud, cloud_out);
    }

    if (path_fincloud_proj.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_fincloud_proj, cloud_projected_out);
    }


    if (path_fincloud.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_fincloud.substr(0,path_fincloud.length()-3)+"inv.ply", cloud_out_inv);
    }
    if (path_fincloud_proj.compare("") != 0) {
        pcl::io::savePLYFileASCII (path_fincloud_proj.substr(0,path_fincloud_proj.length()-3)+"inv.ply", cloud_projected_out_inv);
    }

	// float error = 0;
	// for (int i=0; i<final.points.size(); i++){
	// 	error += sqrt ((final.points[i].x - cloud_image->points[i].x)*(final.points[i].x - cloud_image->points[i].x) 
	// 		          + (final.points[i].y - cloud_image->points[i].y)*(final.points[i].y - cloud_image->points[i].y));
	// }
	// error /= final.points.size();



	// std::cout << "Mean error: " << error << std::endl;




	
	return (0);
}
