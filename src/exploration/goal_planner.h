#pragma once

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 
#include <ctime>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sys/time.h>

#include <nav_msgs/GetMap.h>
#include "geometry_msgs/Pose2D.h"


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"
#include "nav_msgs/OccupancyGrid.h"

#include "projector2d.h"
#include "exploration/fake_projector.h"

#include "frontier_detector.h"
#include "paths_rollout.h"
#include "srrg_types/defs.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalPlanner {

public:

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void velCallback(const geometry_msgs::Twist::ConstPtr& msg);


	GoalPlanner(MoveBaseClient *ac, FakeProjector *projector, FrontierDetector *frontierDetector, cv::Mat *costImage, Vector2f laserOffset = {0.0, 0.5}, int minThresholdSize = 10, std::string mapFrame = "map", std::string baseFrame = "base_link", std::string laserTopicName = "base_scan");

	bool requestOccupancyMap();
	bool requestCloudsUpdate();

	void publishGoal(PoseWithInfo goalPose, std::string frame);

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	Vector2fVector getAbortedGoals();

	void setUnknownCellsCloud(Vector2fVector* cloud);
	void setOccupiedCellsCloud(Vector2fVector* cloud);
	void setMapMetaData(nav_msgs::MapMetaData mapMetaDataMsg);


protected:

	bool isGoalReached();
	void displayStringWithTime(std::string text);
	int computeVisiblePoints(Vector3f robotPose, Vector2f laserOffset);


	FakeProjector *_projector;
	Vector2f _laserOffset;
	FrontierDetector *_frontierDetector;

	nav_msgs::MapMetaData _mapMetaData;

	float _xyThreshold = 0.25;

	PoseWithInfo _goal;
	int _minUnknownRegionSize;


	Vector2fVector* _unknownCellsCloud;
	Vector2fVector* _occupiedCellsCloud;

	Vector2fVector _abortedGoals;

	std::string _mapFrame;
	std::string _baseFrame;
	std::string _laserTopicName;

	cv::Mat* _occupancyMap;
	cv::Mat* _costMap;
	
	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::Subscriber _subLaserScan;
	ros::Subscriber _subVel;

	sensor_msgs::LaserScan _laserscan;
	geometry_msgs::Twist _twist;


	MoveBaseClient* _ac;

	tf::TransformListener _tfListener;
	tf::StampedTransform _tfMapToBase;



};