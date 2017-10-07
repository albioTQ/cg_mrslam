#pragma once

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h> 
#include <algorithm>
#include <stack>
#include "tf/transform_listener.h"

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/GetMap.h>
#include "visualization_msgs/MarkerArray.h"
#include <map_msgs/OccupancyGridUpdate.h>
#include "cloud2d.h"

#include <Eigen/Dense>

#include "srrg_types/defs.h"


using namespace Eigen;
using namespace srrg_core;

typedef std::vector<Vector2iVector> regionVector;

struct coordWithScore {
	Vector2i coord;
	float score;

	bool operator <(const coordWithScore& cws)const
      {
         return score > cws.score;
      }
};



class FrontierDetector {

public:
	void costMapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
	void occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);

	FrontierDetector(cv::Mat *occupancy, cv::Mat *cost, int thresholdSize = 30, int minNeighborsThreshold = 4, std::string namePoints = "points", std::string nameMarkers = "markers", std::string nameMap = "map", std::string nameMapMetadata = "");

	bool requestOccupancyMap();

	void computeFrontiers(int distance = -1, Vector2f centerCoord = {FLT_MAX,FLT_MAX});
		

	void rankRegions();
	
	void computeCentroids();

	void updateClouds();



	Vector2iVector getFrontierPoints();
	regionVector getFrontierRegions();
	Vector2iVector getFrontierCentroids();
	Vector2iVector getUnknownCells();
	Vector2iVector getOccupiedCells();

	Vector2fVector* getUnknownCloud();
	Vector2fVector* getOccupiedCloud();



	nav_msgs::MapMetaData getMapMetaData();



	void publishFrontierPoints();
	void publishCentroidMarkers();




protected:
	Vector2iVector get4Neighbors(Vector2i cell);
	void computeFrontierPoints(int startR, int startC, int endR, int endC);
	void computeFrontierRegions();
	void computeFrontierCentroids();
	void rankFrontierRegions(float mapX, float mapY);


	bool isNeighbor(Vector2i coordI, Vector2i coordJ);
	Vector2iVector getColoredNeighbors(Vector2i coord, int color);
	void createDensePointsCloud(srrg_scan_matcher::Cloud2D *pointCloud, const Vector2iVector points, const bool expansion);



	template<class V, class E> inline bool contains(V vector, E element){
		if (std::find(vector.begin(), vector.end(), element) != vector.end())
			return true;
		else 
			return false;
	}


	cv::Mat  *_occupancyMap;
	cv::Mat *_costMap;


	int _minNeighborsThreshold;
	int _sizeThreshold;
	float _mixtureParam = 1;

	nav_msgs::MapMetaData _mapMetaData;

	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;

	int _circumscribedThreshold = 99;

	Vector2iVector _frontiers;
	regionVector _regions;
	Vector2iVector _centroids;

	Vector2fVector _unknownCellsCloud;
	Vector2fVector _occupiedCellsCloud;

	Vector3f _robotPose;

	std::string _topicPointsName;
	std::string _topicMarkersName;
	std::string _topicMapName;
	std::string _topicMapMetadataName;

	nav_msgs::OccupancyGrid _costMapMsg;

	ros::NodeHandle _nh;
	ros::Publisher _pubFrontierPoints;
	ros::Publisher _pubCentroidMarkers;
	ros::Subscriber _subCostMap;
	ros::Subscriber _subCostMapUpdate;
	ros::Subscriber _subOccupancyMap;
	ros::Subscriber _subOccupancyMapUpdate;
	ros::Subscriber _subActualPose;
	ros::Subscriber _subMapMetaData;
	ros::ServiceClient _mapClient;



	tf::TransformListener _tfListener;
	tf::StampedTransform _tfMapToBase;


};