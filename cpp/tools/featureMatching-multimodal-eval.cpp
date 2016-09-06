#include <iostream>
#include <fstream>
#include <typeinfo>

#include <limits>
#include <vector>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
// #include <pcl/features/organized_edge_detection.h>
// #include <pcl/features/integral_image_normal.h>
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
// #include <edges.h>

#include <cstdlib>

#include <chrono>
#include <SimppleLogger.hpp>

#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>


typedef pcl::PointXYZRGB PointTypeIn;

int i=0;


int readpointcloud (std::string &path, pcl::PointCloud<PointTypeIn> &cloud){
	if (path.substr( path.length() - 3 ) == "pcd")
    {
        if (pcl::io::loadPCDFile<PointTypeIn>(path, cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path);
        }
    }
    else if (path.substr( path.length() - 3 ) == "ply")
    {
        if (pcl::io::loadPLYFile<PointTypeIn>(path, cloud) == -1) {
            PCL_ERROR("Couldn't read the file %s \n", path);
        }
    } else {
        PCL_ERROR("File extension for file %s not recognized\n", path);
    }
    std::cout << "Loaded " << cloud.width << " * " << cloud.height
              << " data points from " << path << std::endl;

}


int
main (int argc, char* argv[])
{
    pplelog::setLogLevel("utils/edges", pplelog::DEBUG);

    std::string c_orig, c_noise1, c_noise2, c_noise1_grad, c_noise2_grad;
    std::string rt;
    float noise;

	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("cloud_orig", boost::program_options::value<std::string>(&c_orig)->required(),
				"Input point cloud in PCD format")
		// ("cloud_noise1", boost::program_options::value<std::string>(&c_noise1)->required(),
		// 		"Input point cloud in PCD format")
		// ("cloud_noise2", boost::program_options::value<std::string>(&c_noise2)->required(),
		// 		"Input point cloud in PCD format")
		("cloud_noise1_grad", boost::program_options::value<std::string>(&c_noise1_grad)->required(),
				"Input point cloud in PCD format")
		("cloud_noise2_grad", boost::program_options::value<std::string>(&c_noise2_grad)->required(),
				"Input point cloud in PCD format")
		("rt", boost::program_options::value<std::string>(&rt)->required())
		("noise", boost::program_options::value<float>(&noise)->required())

		;
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);

	pcl::PointCloud<PointTypeIn>::Ptr cloud_orig(new pcl::PointCloud<PointTypeIn>);
	// pcl::PointCloud<PointTypeIn>::Ptr cloud_noise1(new pcl::PointCloud<PointTypeIn>);
	// pcl::PointCloud<PointTypeIn>::Ptr cloud_noise2(new pcl::PointCloud<PointTypeIn>);
	pcl::PointCloud<PointTypeIn>::Ptr cloud_noise1_grad(new pcl::PointCloud<PointTypeIn>);
	pcl::PointCloud<PointTypeIn>::Ptr cloud_noise2_grad(new pcl::PointCloud<PointTypeIn>);
	readpointcloud(c_orig, *cloud_orig);
	// readpointcloud(c_noise1, *cloud_noise1);
	// readpointcloud(c_noise2, *cloud_noise2);
	readpointcloud(c_noise1_grad, *cloud_noise1_grad);
	readpointcloud(c_noise2_grad, *cloud_noise2_grad);
	int n_points = cloud_orig->width * cloud_orig->height;
	std::cout << "quevoy!" << std::endl;
	rt.erase(0, 1);
	rt.erase(rt.size() - 1);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	std::stringstream str(rt);
	std::vector<float> rt_v((std::istream_iterator<float>(str)),std::istream_iterator<float>());
	std::cout << rt << std::endl;
  	transform.translation() << rt_v[0], rt_v[1], rt_v[2];
  	std::cout << "RT: " <<rt_v[0] << " " << rt_v[1] << " " << rt_v[2] << std::endl;
  	transform.rotate (Eigen::AngleAxisf (rt_v[3],Eigen::Vector3f::UnitX()));	
  	transform.rotate (Eigen::AngleAxisf (rt_v[4],Eigen::Vector3f::UnitY()));
  	transform.rotate (Eigen::AngleAxisf (rt_v[5],Eigen::Vector3f::UnitZ()));	
		std::cout << "quevoy2!" << std::endl;

	pcl::PointCloud<PointTypeIn>::Ptr cloud_orig_transformed(new pcl::PointCloud<PointTypeIn>);
	// pcl::PointCloud<PointTypeIn>::Ptr cloud_noise1_transformed(new pcl::PointCloud<PointTypeIn>);
	pcl::PointCloud<PointTypeIn>::Ptr cloud_noise1_grad_transformed(new pcl::PointCloud<PointTypeIn>);
	std::cout << "quevoy3!" << std::endl;

	pcl::transformPointCloud (*cloud_orig, *cloud_orig_transformed, transform);
	// pcl::transformPointCloud (*cloud_noise1, *cloud_noise1_transformed, transform);
	pcl::transformPointCloud (*cloud_noise1_grad, *cloud_noise1_grad_transformed, transform);
	std::cout << "quevoy4!" << std::endl;

	float distance = 0;
	for (i=0; i<cloud_orig->points.size(); i++){
		float d = (cloud_orig->points[i].x - cloud_orig_transformed->points[i].x) * 
				  (cloud_orig->points[i].x - cloud_orig_transformed->points[i].x);
		     d += (cloud_orig->points[i].y - cloud_orig_transformed->points[i].y) * 
				  (cloud_orig->points[i].y - cloud_orig_transformed->points[i].y);					   
		     d += (cloud_orig->points[i].z - cloud_orig_transformed->points[i].z) * 
				  (cloud_orig->points[i].z - cloud_orig_transformed->points[i].z);	
		distance += sqrt(d);
	}
	distance /= n_points;
	std::cout << "quevo5y!" << std::endl;
	pcl::PointCloud<PointTypeIn> dummy;
	pcl::IterativeClosestPointLine<PointTypeIn, PointTypeIn> icp;
	icp.setMaximumIterations(100);
	icp.setInputSource(cloud_noise2_grad);
	icp.setInputTarget(cloud_noise1_grad_transformed);
	icp.align(dummy);
	std::cout << "has converged:" << icp.hasConverged();
	std::cout << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	// pcl::IterativeClosestPoint<PointTypeIn, PointTypeIn> icp2;
	// icp2.setMaximumIterations(100);
	// icp2.setInputSource(cloud_noise2);
	// icp2.setInputTarget(cloud_noise1_transformed);
	// icp2.align(dummy);
	// std::cout << "has converged:" << icp2.hasConverged() << " score: " <<
	// icp2.getFitnessScore() << std::endl;
	// std::cout << icp2.getFinalTransformation() << std::endl;

	pcl::PointCloud<PointTypeIn>::Ptr cloud_transformed_mine(new pcl::PointCloud<PointTypeIn>);
	// pcl::PointCloud<PointTypeIn>::Ptr cloud_transformed_icp(new pcl::PointCloud<PointTypeIn>);

	pcl::transformPointCloud (*cloud_orig_transformed, *cloud_transformed_mine, (icp.getFinalTransformation()).inverse().eval());
	// pcl::transformPointCloud (*cloud_orig_transformed, *cloud_transformed_icp, (icp2.getFinalTransformation()).inverse().eval());


	float distance_mine = 0;
	for (i=0; i<cloud_orig->points.size(); i++){
		PointTypeIn p1, p2;
		p1.x = cloud_orig->points[i].x / cloud_orig->points[i].z;
		p1.y = cloud_orig->points[i].y / cloud_orig->points[i].z;
		p2.x = cloud_transformed_mine->points[i].x / cloud_transformed_mine->points[i].z;
		p2.y = cloud_transformed_mine->points[i].y / cloud_transformed_mine->points[i].z;
		float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
		distance_mine += sqrt(d);
	}
	distance_mine /= n_points;

	// float distance_icp = 0;
	// for (i=0; i<cloud_orig->points.size(); i++){
	// 	PointTypeIn p1, p2;
	// 	p1.x = cloud_orig->points[i].x / cloud_orig->points[i].z;
	// 	p1.y = cloud_orig->points[i].y / cloud_orig->points[i].z;
	// 	p2.x = cloud_transformed_icp->points[i].x / cloud_transformed_icp->points[i].z;
	// 	p2.y = cloud_transformed_icp->points[i].y / cloud_transformed_icp->points[i].z;
	// 	float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
	// 	distance_icp += sqrt(d);
	// }
	// distance_icp /= n_points;
 
	Eigen::Matrix3f m = icp.getFinalTransformation().block(0,0, 3, 3).inverse() * 
		transform.matrix().eval().eval().block(0,0, 3, 3);
	Eigen::AngleAxisf difference_angle(m);
	Eigen::Matrix3f m2 = transform.matrix().eval().block(0,0, 3, 3);
	Eigen::AngleAxisf original_angle(m2);
	// Eigen::AngleAxisf difference_angle(transform.matrix().eval().block(0,0, 3, 3).inverse() * 
	// 	icp.getFinalTransformation().eval().block(0,0, 3, 3));    // RotationMatrix to AxisAngle

	float difference_rotation = (transform.matrix().eval().block(0,3,3,1) -
	   icp.getFinalTransformation().eval().block(0,3,3,1)).norm();
	float original_rotation = (transform.matrix().eval().block(0,3,3,1)).norm();

	// std::cout << "RESULTS " << noise <<" " <<  distance << " " << distance_mine << " " << distance_icp << std::endl;
	std::cout << "RESULTS_OTHER " << noise << " " 
			  // << std::to_string(rt_v[0]) << " "
			  // << std::to_string(rt_v[1])<< " "  
			  // << std::to_string(rt_v[2])<< " "  
			  // << initial.axis() << " " 
			  // << std::to_string(initial.angle()) << " "
			  // << std::to_string((icp.getFinalTransformation().inverse().eval())(0,3)) << " " << 
			  // << std::to_string((icp.getFinalTransformation().inverse().eval())(1,3)) << " " << 
			  // << std::to_string((icp.getFinalTransformation().inverse().eval())(2,3)) << " " << 
			  // << final.axis() << " " 
			  << std::to_string(original_angle.angle()) << " " 
			  << std::to_string(original_rotation) << " "
			  << std::to_string(difference_angle.angle()) << " " 
			  << std::to_string(difference_rotation) << " " 
			  << std::to_string(distance_mine) << std::endl;


	return (0);
}
