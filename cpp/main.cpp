#include <iostream>

#include <limits>
#include <vector>

#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>

#include <opencv2/opencv.hpp>

#include <opencv2pcl.h>
#include <csvreader.h>


typedef pcl::PointXYZRGB PointTypeIn;
typedef pcl::PointXYZI PointTypeOut;

int
main ()
{
    // Define external paths
    //std::string path = "/opt/Datasets/IMPART/7-Multi-modal/Kitchen/LIDAR/Kitchen-Lidar-RoughMerge.ply";
    //std::string path = "/opt/Datasets/IMPART/7-Multi-modal/Lobby/LIDAR/Ground-Lidar-rough-merge.ply";
    std::string path = "/opt/Datasets/2015-11-27-3/pcd/frame_000000.pcd";

    std::string imagepath = "/opt/Datasets/2015-11-27-3/images/0.jpg";
	//std::string imagepath = "/opt/Datasets/recording-oni/images/IMG_20151125_151247.jpg";
//
//	// Working with image
	cv::Mat src_full, src, src_gray, detected_edges, dst;
    src_full = cv::imread(imagepath, 1);
    cv::resize(src_full, src, cv::Size(src_full.cols/4, src_full.rows/4));
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );
    cv::Canny(src_gray, detected_edges, 40, 100);
    dst = cv::Scalar::all(0);
    src.copyTo( dst, detected_edges);

    cv::imshow( "Test", dst );
    cv::imwrite("orig.png", src);
    cv::imwrite("result.png", dst);
    cv::waitKey(0);

    // Working with point clouds
    // Define initial cloud and keypoints
    pcl::PointCloud<PointTypeIn>::Ptr cloud(new pcl::PointCloud<PointTypeIn>);
	if (pcl::io::loadPCDFile<PointTypeIn>(path, *cloud)==-1)
	{
		PCL_ERROR ("Couldn't read the file %s \n", path);
		return (-1);
	}    std::cout << "Loaded "
		  << cloud->width << " * " <<  cloud->height
		  << " data points from " << path
		  << std::endl;

	  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	  pcl::IntegralImageNormalEstimation<PointTypeIn, pcl::Normal> ne;
	  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	  ne.setNormalSmoothingSize (10.0f);
	  ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
	  ne.setInputCloud (cloud);
	  ne.compute (*normal);

	  float default_th_dd = 0.1f;
	  int   default_max_search = 50;
	  pcl::OrganizedEdgeFromRGBNormals<PointTypeIn, pcl::Normal, pcl::Label> oed;
	  oed.setInputNormals (normal);
	  oed.setInputCloud (cloud);
	  oed.setDepthDisconThreshold (default_th_dd);
	  oed.setHCCannyHighThreshold(1.5);
	  oed.setHCCannyLowThreshold(1.1);
	  oed.setMaxSearchNeighbors (default_max_search);
	  oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
	  pcl::PointCloud<pcl::Label> labels;
	  std::vector<pcl::PointIndices> label_indices;
	  oed.compute (labels, label_indices);

	 pcl::visualization::PCLVisualizer viewer ("3D Edge Viewer");
	  // Mak1.5e gray point clouds
//	  for (size_t idx = 0; idx < cloud->points.size (); idx++)
//	  {
//	    uint8_t gray = uint8_t ((cloud->points[idx].r + cloud->points[idx].g + cloud->points[idx].b) / 3);
//	    cloud->points[idx].r = cloud->points[idx].g = cloud->points[idx].b = gray;
//	  }

	  // Display edges in PCLVisualizer
	  viewer.setSize (640, 480);
	  viewer.addCoordinateSystem (0.2f, "global");
	  viewer.addPointCloud (cloud, "original point cloud");

	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	    occluded_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	    nan_boundary_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	    high_curvature_edges (new pcl::PointCloud<pcl::PointXYZRGBA>),
	    rgb_edges (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  pcl::copyPointCloud (*cloud, label_indices[0].indices, *nan_boundary_edges);
	  pcl::copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
	  pcl::copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
	  pcl::copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
	  pcl::copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);

	  const int point_size = 2;
//	  viewer.addPointCloud<pcl::PointXYZRGBA> (nan_boundary_edges, "nan boundary edges");
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "nan boundary edges");
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.0f, 1.0f, "nan boundary edges");

	  viewer.addPointCloud<pcl::PointXYZRGBA> (occluding_edges, "occluding edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluding edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 0.0f, "occluding edges");

//	  viewer.addPointCloud<pcl::PointXYZRGBA> (occluded_edges, "occluded edges");
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "occluded edges");
//	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0.0f, 0.0f, "occluded edges");

	  viewer.addPointCloud<pcl::PointXYZRGBA> (high_curvature_edges, "high curvature edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "high curvature edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 1.0f, 0.0f, "high curvature edges");

	  viewer.addPointCloud<pcl::PointXYZRGBA> (rgb_edges, "rgb edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "rgb edges");
	  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 1.0f, 1.0f, "rgb edges");

	  viewer.saveScreenshot("screenshot.png");

	  pcl::visualization::Camera cam;
	  while (!viewer.wasStopped ())
	  {
		viewer.getCameraParameters(cam);
		std::cout << "Focal: " << cam.focal[0] << " " << cam.focal[1] << " " << cam.focal[2] << std::endl;
		std::cout << "Pos: " << cam.pos[0] << " " << cam.pos[1] << " " << cam.pos[2] << std::endl;
		std::cout << "View: " << cam.view[0] << " " << cam.view[1] << " " << cam.view[2] << std::endl;
		std::cout << "Clip: " << cam.clip[0] << " " << cam.clip[1] << std::endl;
		std::cout << "Fovy: " << cam.fovy << std::endl;
		std::cout << "Window_size: " << cam.window_size[0] << " " << cam.window_size[1] << std::endl;
		std::cout << "Window_pos: " << cam.window_pos[0] << " " << cam.window_pos[1]  << std::endl;
	    viewer.spinOnce ();
	    boost::this_thread::sleep (boost::posix_time::microseconds (100));
	  }




//    pcl::visualization::CloudViewer viewer1 ("All cloud");
//    viewer1.showCloud (cloud);
//    while (!viewer1.wasStopped ()){}

    return (0);
}
