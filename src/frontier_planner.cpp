
#include <unistd.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"

#include <sys/time.h>

using namespace srrg_core;
using namespace Eigen;

double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) -
    (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

int main (int argc, char **argv){


std::string frontierPointsTopic, markersTopic;

int thresholdRegionSize;
int thresholdExploredArea;
nav_msgs::MapMetaData occupancyMapInfo;

double lambdaDecay;

int maxCentroidsNumber;
double farCentroidsThreshold;
double nearCentroidsThreshold;
int numExplorationIterations;

Vector2iVector centroids;
Vector2iVector frontierPoints;
Vector2fVector abortedGoals;
regionVector regions;
Vector2fVector *unknownCellsCloud, *occupiedCellsCloud;
 

std::string mapFrame, baseFrame, laserFrame, laserTopicName;

cv::Mat occupancyMap, costMap;


Vector2f laserOffset;

ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh("~");

nh.param("mapFrame", mapFrame, std::string("map"));
nh.param("baseFrame", baseFrame, std::string("base_link"));
nh.param("laserFrame", laserFrame, std::string("base_laser_link"));
nh.param("scanTopic", laserTopicName, std::string("scan"));
nh.param("pointsTopic", frontierPointsTopic, std::string("points"));
nh.param("markersTopic", markersTopic, std::string("markers"));
nh.param("regionSize", thresholdRegionSize, 15);
nh.param("exploredArea", thresholdExploredArea, 10);
nh.param("lambda", lambdaDecay, 1.25);
nh.param("mc", nearCentroidsThreshold, 0.5);
nh.param("Mc", farCentroidsThreshold, 5.0);
nh.param("nc", maxCentroidsNumber, 8);
nh.param("iter", numExplorationIterations, -1);



//Laserscan FAKE projection parameters
float minRange = 0.0;
float maxRange = 4.0;
int numRanges = 181;
float fov = M_PI;
Vector2f rangesLimits = {minRange, maxRange};

FakeProjector projector;
projector.setMaxRange(maxRange);
projector.setMinRange(minRange);
projector.setFov(fov);
projector.setNumRanges(numRanges);

MoveBaseClient ac("move_base",true);
ac.waitForServer(); //will wait for infinite time

tf::TransformListener tfListener;
tf::StampedTransform tfBase2Laser;
try{
	tfListener.waitForTransform(baseFrame, laserFrame, ros::Time(0), ros::Duration(1.0));
	tfListener.lookupTransform(baseFrame, laserFrame, ros::Time(0), tfBase2Laser);

	laserOffset  = {tfBase2Laser.getOrigin().x(), tfBase2Laser.getOrigin().y()}; 
}
catch (...) {
	laserOffset = {0.05, 0.0};
	std::cout<<"Catch exception: "<<laserFrame<<" not exists. Using default values."<<std::endl;
 }

FrontierDetector frontiersDetector(&occupancyMap, &costMap,thresholdRegionSize);

unknownCellsCloud = frontiersDetector.getUnknownCloud();
occupiedCellsCloud = frontiersDetector.getOccupiedCloud();

PathsRollout pathsRollout(&costMap, &ac, &projector, laserOffset, maxCentroidsNumber, thresholdExploredArea, nearCentroidsThreshold, farCentroidsThreshold, 1, 8, lambdaDecay);

pathsRollout.setUnknownCellsCloud(unknownCellsCloud);
pathsRollout.setOccupiedCellsCloud(occupiedCellsCloud);

GoalPlanner goalPlanner(&ac, &projector, &frontiersDetector, &costMap, laserOffset, thresholdExploredArea, mapFrame, baseFrame,laserTopicName);

goalPlanner.setUnknownCellsCloud(unknownCellsCloud);
goalPlanner.setOccupiedCellsCloud(occupiedCellsCloud);


std::cout<<"EXPLORATION: "<<std::endl;
std::cout<<"ITERATIONS: "<< numExplorationIterations<<std::endl;
 
while (ros::ok() && (numExplorationIterations != 0)){


	frontiersDetector.computeFrontiers();

	frontiersDetector.publishFrontierPoints();
   	frontiersDetector.publishCentroidMarkers();

	frontierPoints = frontiersDetector.getFrontierPoints();
	regions = frontiersDetector.getFrontierRegions();
	centroids = frontiersDetector.getFrontierCentroids();


	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS EXTRACTED... EXIT"<<std::endl;
		return 0;
	}

	else {

		occupancyMapInfo = frontiersDetector.getMapMetaData();
		pathsRollout.setMapMetaData(occupancyMapInfo);
		goalPlanner.setMapMetaData(occupancyMapInfo);

		abortedGoals = goalPlanner.getAbortedGoals();
		pathsRollout.setAbortedGoals(abortedGoals);

		int numSampledPoses = pathsRollout.computeAllSampledPlans(centroids, mapFrame);
		if (numSampledPoses == 0){
			std::cout<<"NO POSE AVAILABLE FOR GOAL... EXIT"<<std::endl;
			return 0;
		}

		PoseWithInfo goal = pathsRollout.extractBestPose();

		std::string frame = mapFrame;

		goalPlanner.publishGoal(goal, frame);
		goalPlanner.waitForGoal();

		numExplorationIterations--;
	}


}

	return 0;
}
