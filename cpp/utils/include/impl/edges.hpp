/*
 * edges.hpp
 *
 *  Created on: Dec 4, 2015
 *      Author: alba
 */

#include <edges.h>
#include <SimppleLogger.hpp>
pplelog::SimppleLogger *logger = pplelog::getLogger("utils/edges");
pplelog::SimppleLogger *timelogger = pplelog::getLogger("timer");


using namespace utils;

template<typename PointT>
void LineCloud<PointT>::populateCloud(pcl::PointCloud<PointT> &cloud){
    std::random_shuffle(cloud.points.begin(), cloud.points.end());
	std::vector<bool> cloud_used(cloud.points.size(), false);
	std::vector<bool> cloud_needed(cloud.points.size(), true);
	for (int j = 0; j < cloud.points.size(); j++) {
			if(cloud.points[j].intensity < weight_thresh){
				cloud_needed[j]=false;
		}
	}
	// ToDo: cloud_threshold to know points above threshold;
	std::queue<int> search_branches;
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(typename pcl::PointCloud<PointT>::Ptr(&cloud));
	auto it = std::find(cloud_needed.begin(), cloud_needed.end(),true);
	while (it < cloud_needed.end()){ // Search all lines on image
		int elem = std::distance(cloud_needed.begin(), it);
	    SPPLELOG_TRACE(logger, elem << " " << cloud.points[elem]);
		// Start new line
	    SPPLELOG_TRACE(logger, "Starting a new line");
		LinePoint<PointT> line(line_threshold);
		line.tryToPut(cloud.points[elem]);
		*it = false;
	    SPPLELOG_TRACE(logger, "Putting point " << elem << " on the line");

		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		kdtree.nearestKSearch (cloud.points[elem], K, pointIdxNKNSearch, pointNKNSquaredDistance);
	    SPPLELOG_TRACE(logger, "Found " << pointIdxNKNSearch.size()  << " neighbours:");
		for (int i=0; i<pointIdxNKNSearch.size(); i++){
	    	SPPLELOG_TRACE(logger, pointIdxNKNSearch[i] << ": distance " << pointNKNSquaredDistance[i] <<
	    			" (threshold " << neigh_threshold << ", used "<< cloud_used[pointIdxNKNSearch[i]] <<" )");
		}

		for (size_t i=0; i<pointIdxNKNSearch.size(); i++){
			if (pointNKNSquaredDistance[i] > 0 &&
					pointNKNSquaredDistance[i] < neigh_threshold &&
					!cloud_used[pointIdxNKNSearch[i]]){
				search_branches.push(pointIdxNKNSearch[i]);
			    SPPLELOG_TRACE(logger, "Adding point " << pointIdxNKNSearch[i] << " to the search branch");
			}
		}
		SPPLELOG_TRACE(logger, "There are " << search_branches.size() << "points to search");
		while (!search_branches.empty()) {
			SPPLELOG_TRACE(logger, "There are " << search_branches.size() << "points to search");
		    SPPLELOG_TRACE(logger, "Trying to put point " <<search_branches.front() << " in the line");
			if (!cloud_used[search_branches.front()] && line.tryToPut(cloud.points[search_branches.front()])) {
			    SPPLELOG_TRACE(logger, "Point in line!");
				// If the element is put, search the surroundings and put them on the queue
				cloud_used[search_branches.front()] = true;
				cloud_needed[search_branches.front()] = false;
				std::cout << "point less" << std::endl;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				kdtree.nearestKSearch(cloud.points[search_branches.front()], K, pointIdxNKNSearch,
						pointNKNSquaredDistance);
			    SPPLELOG_TRACE(logger, "Found " << pointIdxNKNSearch.size()  << " neighbours:");
				for (int i=0; i<pointIdxNKNSearch.size(); i++){
				    SPPLELOG_TRACE(logger, pointIdxNKNSearch[i] << ": distance " << pointNKNSquaredDistance[i] <<
				    		" (threshold " << neigh_threshold << " ) ");
				}
				for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
					if (pointNKNSquaredDistance[i] > 0
							&& pointNKNSquaredDistance[i] < neigh_threshold &&
							!cloud_used[pointIdxNKNSearch[i]]){
						search_branches.push(pointIdxNKNSearch[i]);
					    SPPLELOG_TRACE(logger,"Adding point " << pointIdxNKNSearch[i] << " to the search branch" );
					}
				}
			}

		    SPPLELOG_TRACE(logger,  "popping point..." );
			search_branches.pop();
		}

		SPPLELOG_TRACE(logger, "Line detected!");
		//lines.push_back(line);
		this->add_line(line);
		it = std::find(cloud_needed.begin(), cloud_needed.end(),true);
	}
}

template<typename PointT>
void LineCloud<PointT>::add_line(LinePoint<PointT> line_in){
	if (!lines.empty()){
		bool merged=false;
		std::vector<double> dists;
		for (auto& line:lines){
			dists.push_back(line.compare_line(line_in));
		}
		auto min = std::min_element(dists.begin(), dists.end());;
		SPPLELOG_TRACE(logger, "The minimum distance is " << *min);
		if (*min > compare_line_threshold){
			lines.push_back(line_in);
		} else {
			lines[min-dists.begin()].mergeLines(line_in);
		}
	} else {
		lines.push_back(line_in);
	}
}


template<typename PointT>
double LinePoint<PointT>::compare_line(LinePoint<PointT> line){
	arma::vec v1(3), v2(3), x1(3), x2(3);
	// Assign all elements
	v1[0] = this->vec[0]; v1[1] = this->vec[1]; v1[2] = this->vec[2];
	x1[0] = this->orig[0]; x1[1] = this->orig[1]; x1[2] = this->orig[2];
	v2[0] = line.vec[0]; v2[1] = line.vec[1]; v2[2] = line.vec[2];
	x2[0] = line.orig[0]; x2[1] = line.orig[1]; x2[2] = line.orig[2];
	// Compute both distances
	double sep = arma::norm(arma::cross(v1,v2),2);
	double dist = arma::norm(arma::cross(x1-x2, v1),2);
	return sep+dist;
}

template<typename PointT>
bool LinePoint<PointT>::tryToPut(PointT point){
	if (points.size() < 1){
		SPPLELOG_TRACE(logger, "Only point. Putting!");
		points.push_back(point);
		estimateExtremes(point);
		return true;
	} else if (points.size() < 2){
		SPPLELOG_TRACE(logger, "Two points. Putting!");
		points.push_back(point);
		estimateParameters();
		estimateExtremes(point);
		return true;
	} else if (testFitting(point)){
		SPPLELOG_TRACE(logger, "3+ points. It fits!");
		points.push_back(point);
		estimateParameters();
		estimateExtremes(point);
		return true;
	} else {
		SPPLELOG_TRACE(logger, "No fit!");
		return false;
	}
}

template<typename PointT>
void LinePoint<PointT>::mergeLines(LinePoint<PointT> line){
	points.insert(points.end(), line.points.begin(), line.points.end());
	estimateParameters();
}


template<typename PointT>
void LinePoint<PointT>::estimateParameters(){
	SPPLELOG_TRACE(logger, "Estimating parameters of the line");
	int n = points.size();
	orig[0]=0; orig[1]=0; orig[2]=0;
	arma::mat X(n,3), U, V;
	arma::vec s, avg_v(3);

	for (auto& p: points){
		orig[0] += p.x;
		orig[1] += p.y;
		orig[2] += p.z;
	}
	orig[0] /=n; orig[1] /=n; orig[2] /=n;
	avg_v(0)=orig[0]; avg_v(1)=orig[1]; avg_v(2)=orig[2];

	for (int i=0; i<n; i++){
		SPPLELOG_TRACE(logger,  "Point" << points[i]);
		X(i,0)=points[i].x-orig[0];
		X(i,1)=points[i].y-orig[1];
		X(i,2)=points[i].z-orig[2];
	}

	arma::svd(U,s,V,X);
	SPPLELOG_TRACE(logger, "svd done " << V);

	vec[0]=V(0,0); vec[1]=V(1,0); vec[2]=V(2,0);
	SPPLELOG_TRACE(logger, "Computed parameters: " <<
			"[" << orig[0] << ", " << orig[1] << ", " << orig[2] << "]" <<
			"[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]");
	min_t=0, max_t=0; 
	for (auto& p: points){
		double t = (p.x +p.y+p.z - orig[0]-orig[1]-orig[2])/(vec[0]+vec[1]+vec[2]);
		if (t<min_t) min_t=t;
		if (t>max_t) max_t=t;
	}
}
template<typename PointT>
void LinePoint<PointT>::estimateExtremes(PointT point){
}


template<typename PointT>
bool LinePoint<PointT>::testFitting(PointT point){
	arma::vec q(3), u(3), p(3);
	q(0) = orig[0]; q(1)=orig[1]; q(2)=orig[2];
	u(0) = vec[0]; u(1)=vec[1]; u(2)=vec[2];
	p(0) = point.x; p(1)=point.y; p(2)=point.z;

	double dist;
	dist = arma::norm(arma::cross(p-q, u),2);

	SPPLELOG_TRACE(logger,  "Point " << point << " is at distance " << dist  <<
			" (max " << threshold << " )");
	if (dist > threshold){
		return false;
	}
	return true;
}

template<typename PointT>
void LineCloud<PointT>::filter_lines_by_size(int size){
	std::vector<LinePoint<PointT> > lines_new;
	for (auto& line:lines){
		if (line.points.size()>size){
			lines_new.push_back(line);
		}
	}
	lines.erase(lines.begin(), lines.end());
	lines = lines_new;

}

template<typename PointT>
void LineCloud<PointT>::filter_lines_by_disparity(double disparity){
//	std::vector<LinePoint<PointT> > lines_new;
//	for (auto& line:lines){
//		double disp = 0;
//		for (auto point:line.points){
//
//		}
//		disp/=line.points.size();
//		std::cout << "The disparity of the line is " << disp << std::endl;
//		if (disp<disparity){
//			lines_new.push_back(line);
//		}
//	}
//	std::cout << "Liiines " << lines.size() << std::endl;
//
//	lines.erase(lines.begin(), lines.end());
//	lines = lines_new;

}

template<typename PointT>
void LineCloud<PointT>::write(std::string &filename){
	std::ofstream out;
	out.open(filename);
	for(int i=0; i<lines.size(); i++){
		out << lines[i].vec[0] << " " << lines[i].vec[1] << " " 
		    << lines[i].vec[2] << " " << lines[i].orig[0] << " "  
		    << lines[i].orig[1] << " " << lines[i].orig[2] << " "
		    << lines[i].min_t << " " << lines[i].max_t<< '\n';
	}
	out.close();
}

template<typename PointT>
void LineCloud<PointT>::read(std::string &filename){
	std::ifstream in;
	in.open(filename);
	double x0, y0, z0, v1, v2, v3, tmin, tmax;
	while (in >> v1 >> v2 >> v3 >> x0 >> y0 >> z0 >> tmin >> tmax){
		LinePoint<PointT> line(0);
		line.vec[0] = v1; line.vec[1] = v2; line.vec[2] = v3;
		line.orig[0] = x0; line.orig[1] = y0; line.orig[2] = z0;
		line.min_t = tmin; line.max_t = tmax;  
		lines.push_back(line);
	}
}