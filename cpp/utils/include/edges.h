/*
 * edges.h
 *
 *  Created on: Dec 4, 2015
 *      Author: alba
 */

#ifndef UTILS_INCLUDE_EDGES_H_
#define UTILS_INCLUDE_EDGES_H_

#include <pcl/pcl_base.h>
#include <iostream>
#include <armadillo>
#include <math.h>

namespace utils {
	template<typename PointT> class LinePoint {
		public:
			LinePoint(): threshold(1.0){
			}
			LinePoint(double threshold):
					threshold(threshold){
			}
			bool tryToPut(PointT point);
			void estimateParameters();
			void estimateExtremes(PointT point);
			bool testFitting(PointT point);
			void mergeLines(LinePoint<PointT> line);
			double compare_line(LinePoint<PointT> line);

			pcl::PointCloud<PointT> points; // Points included in the line
			//PointT p_min, p_max; // Extremes of the line
			//double tx_min, tx_max, ty_min, ty_max; // Extremes of the line -- related
			double vec[3], orig[3]; // Parameters of a line in 3D
			double min_t, max_t; // Distance to the center of the extreme points
			double threshold; // Distance from a point to be considered for the line
	};

	template<typename PointT> class LineCloud {
		public:
			LineCloud(): line_threshold(1.0), neigh_threshold(1.0), compare_line_threshold(1.0), 
					K(2), weight_thresh(0.0){
			}
			LineCloud(int K, double neigh_threshold, double line_threshold, 
					double compare_line_threshold, float weight_thresh):
					K(K),line_threshold(line_threshold), neigh_threshold(neigh_threshold),
					compare_line_threshold(compare_line_threshold), weight_thresh(weight_thresh){
			}
			int K;
			double neigh_threshold;
			double line_threshold; // Distance from a point to be considered for the line
			double compare_line_threshold;
			float weight_thresh;

			std::vector<LinePoint<PointT> > lines;
			void populateCloud(pcl::PointCloud<PointT> & cloud);
			void add_line(LinePoint<PointT> line_in);
			void filter_lines_by_size(int size=30);
			void filter_lines_by_disparity(double disparity=0);
			void write(std::string &filename);
			void read(std::string &filename);
	};

}

#include <impl/edges.hpp>

#endif /* UTILS_INCLUDE_EDGES_H_ */
