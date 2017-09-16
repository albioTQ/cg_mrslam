
#include <unistd.h>

#include "g2o/stuff/command_args.h"

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

g2o::CommandArgs arg;

std::string frontierPointsTopic, markersTopic;

int thresholdRegionSize;
int thresholdExploredArea;
nav_msgs::MapMetaData occupancyMapInfo;

float lambdaDecay;

int maxCentroidsNumber;
float farCentroidsThreshold;
float nearCentroidsThreshold;
int numExplorationIterations;

Vector2iVector centroids;
Vector2iVector frontierPoints;
Vector2fVector abortedGoals;
regionVector regions;
Vector2fVector *unknownCellsCloud, *occupiedCellsCloud;
 

std::string mapFrame, baseFrame, laserFrame, laserTopicName;

cv::Mat occupancyMap, costMap;


Vector2f laserOffset;

arg.param("mapFrame", mapFrame, "map", "TF mapFrame for the robot");
arg.param("baseFrame", baseFrame, "base_link", "TF base Frame for the robot");
arg.param("laserFrame", laserFrame, "base_laser_link", "TF laser Frame for the robot");
arg.param("scanTopic", laserTopicName,"scan", "laser scan ROS topic");
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.param("exploredArea", thresholdExploredArea, 10, "minimum number of visible frontier points before aborting a goal");
arg.param("lambda", lambdaDecay, 1.25, "distance decay factor for choosing next goal");
arg.param("mc", nearCentroidsThreshold, 0.5, "Lower distance limit to consider 2 goals as the same, in meters");
arg.param("Mc", farCentroidsThreshold, 5.0, "Max distance at which a centroid is considered if there are also closer ones, in meters");
arg.param("nc", maxCentroidsNumber, 8, "Maximum number of centroids considered during each search for a goal");
arg.param("iter", numExplorationIterations, 10, "Number of plans to be computed. -1 means infinite");

arg.parseArgs(argc, argv);

ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;

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
