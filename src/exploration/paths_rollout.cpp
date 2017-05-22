#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;



PathsRollout::PathsRollout(cv::Mat* costMap, MoveBaseClient *ac, FakeProjector *projector, Vector2f laserOffset, int maxCentroidsNumber, int regionSize, float nearCentroidsThreshold, float farCentroidsThreshold, float sampleThreshold, int sampleOrientation, float lambdaDecay){

	_nearCentroidsThreshold = nearCentroidsThreshold;
	_farCentroidsThreshold = farCentroidsThreshold;

	_laserOffset = laserOffset;

	_costMap = costMap;

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;
	_sampleOrientation = sampleOrientation;
	_intervalOrientation = 2*M_PI/sampleOrientation;

	_maxCentroidsNumber = maxCentroidsNumber;
	_minUnknownRegionSize = regionSize;
	
	_projector = projector;

	_ac = ac;

	_lambda = lambdaDecay;

	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");


}





int PathsRollout::computeAllSampledPlans(Vector2iVector centroids, std::string frame){

	_vectorSampledPoses.clear();
	Vector2fVector meterCentroids;
	int countPlans = 0;


	for (int i = 0; i < centroids.size(); i ++){
		Vector2f meterCentroid = {centroids[i][1]* _resolution + _mapOriginY, centroids[i][0]* _resolution + _mapOriginX};//Inverted because computed in map (row col -> y x)
		meterCentroids.push_back(meterCentroid); 	
	}

	try{
	_tfListener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
	_tfListener.lookupTransform("map", "base_link", ros::Time(0), _tfMapToBase);

	}
	catch (...) {
		std::cout<<"Catch exception: map2odom tf exception... Using old values."<<std::endl;
	 }

	geometry_msgs::Pose startPose;
	startPose.position.x = _tfMapToBase.getOrigin().x(); 
	startPose.position.y = _tfMapToBase.getOrigin().y();

	std::cout<<startPose.position.x <<" "<<startPose.position.y<<std::endl;

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(_tfMapToBase.getRotation(),qMsg);

	startPose.orientation = qMsg;  


	//Wait for robot to stop moving (moveBaseClient can't do 2 things at same time)
	ros::Rate checkReadyRate(100);
	while(!isActionDone(_ac)){
		checkReadyRate.sleep();
	}

	_longestPlan = 0;
	
	for (int i = 0; i < meterCentroids.size(); i++){


		geometry_msgs::Pose goalPose;
		goalPose.position.x = meterCentroids[i][0];  
		goalPose.position.y = meterCentroids[i][1];

			std::cout<<goalPose.position.x <<" "<<goalPose.position.y<<std::endl;


		float distanceActualPose = sqrt(pow((meterCentroids[i][0] - startPose.position.x),2) + pow((meterCentroids[i][1] - startPose.position.y),2));

		if ((countPlans > (round(_maxCentroidsNumber/2)))&&(distanceActualPose > _farCentroidsThreshold)){ //If I have already some plans and this is quite far, break.
				break;
			}

		std::vector<PoseWithInfo> sampledPlan = makeSampledPlan(frame, startPose, goalPose);

		if (sampledPlan.size()>0){
			countPlans++;
		}
		else {
			continue;
		}

		if (sampledPlan[0].planLenght > _longestPlan){
			_longestPlan = sampledPlan[0].planLenght;
		}

		//Loop through sampled poses to check if I am already considering them from previous sampled plans
		for (int j = 0; j < sampledPlan.size(); j++){

			bool farAlreadySampledPoses = true;

			for (int k = 0; k < _vectorSampledPoses.size(); k++){
				float distanceX = fabs(_vectorSampledPoses[k].pose[0] - sampledPlan[j].pose[0]);
				float distanceY = fabs(_vectorSampledPoses[k].pose[1] - sampledPlan[j].pose[1]);

				if ((distanceX <= _xyThreshold) &&(distanceY <= _xyThreshold)){
					farAlreadySampledPoses = false;
					break;
				}

			}

			if (farAlreadySampledPoses){
				_vectorSampledPoses.push_back(sampledPlan[j]);
			}

		}


		if (countPlans == _maxCentroidsNumber){ //When I reach the limit I stop computing plans
			break; 		}


	}


	return _vectorSampledPoses.size();

}

PoseWithInfo PathsRollout::extractGoalFromSampledPoses(){


	PoseWithInfo goalPose;

	Cloud2D augmentedCloud = *_unknownCellsCloud;

	augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());

	
	goalPose = extractBestPose(augmentedCloud);


	return goalPose;

}




std::vector<PoseWithInfo> PathsRollout::makeSampledPlan( std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){


	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;
	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	std::vector<PoseWithInfo> sampledPlan;

	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {

        	 sampledPlan = sampleTrajectory(res.plan);
            }
    }
    else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", _planClient.getService().c_str());
    }

    return sampledPlan;

}







std::vector<PoseWithInfo> PathsRollout::sampleTrajectory(nav_msgs::Path path){

	std::vector<PoseWithInfo> vecSampledPoses;

	if (!path.poses.empty()){

		PoseWithInfo lastPose;
		//This is the starting pose
		double initialAngle = tf::getYaw(_tfMapToBase.getRotation());
		lastPose.pose = {_tfMapToBase.getOrigin().x(), _tfMapToBase.getOrigin().y(), initialAngle};
		lastPose.predictedAngle = initialAngle;
		lastPose.index = 0;
		lastPose.planLenght = path.poses.size() - 1;

		//bool nearToAborted = false;
		//for (int j = 0; j < _abortedGoals.size(); j ++){
		//		float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - lastPose.pose[0]),2) + pow((_abortedGoals[j][1] - lastPose.pose[1]),2));
		//	if (distanceAbortedGoal < _nearCentroidsThreshold){
		//		nearToAborted = true;
		//		break;
		//	}
		//}
		//if (!nearToAborted){
			vecSampledPoses.push_back(lastPose);
		//}
		//Given a non empty path, I sample it (first pose already sampled, last pose will be treated differently later)
		for (int i = 1; i < path.poses.size() - 1; i++){

			PoseWithInfo newPose;

			//NOTE: The orientation of this pose will be eventually computed if the pose will be sampled
			newPose.pose = {path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0.0};
			Vector2i newPoseMap = {round((newPose.pose[0] - _mapOriginX)/_resolution), round((newPose.pose[1] - _mapOriginY)/_resolution)};
			float distancePreviousPose = sqrt(pow((lastPose.pose[0] - newPose.pose[0]),2) + pow((lastPose.pose[1] - newPose.pose[1]),2));
			//If the pose I'm considering is quite far from the lastPose sampled and it's not too close to an obstacle I proceed
			if ((distancePreviousPose >= _sampledPathThreshold)&&(_costMap->at<unsigned char>(newPoseMap[1], newPoseMap[0]) < _circumscribedThreshold)){
				bool nearToAborted = false;
				//Check if the newPose is too close to a previously aborted goal.
				for (int j = 0; j < _abortedGoals.size(); j ++){
					float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose.pose[0]),2) + pow((_abortedGoals[j][1] - newPose.pose[1]),2));
					if (distanceAbortedGoal < _nearCentroidsThreshold){
						nearToAborted = true;
						break;
					}
				}

				if (!nearToAborted){

					float predictedAngle = predictAngle(lastPose.pose, newPose.pose);

					//Fill some fields of PoseWithInfo
					newPose.pose[2] = predictedAngle;
					newPose.predictedAngle = predictedAngle;
					newPose.index = i;
					newPose.planLenght = path.poses.size() - 1;
					//Sample the NewPose
					vecSampledPoses.push_back(newPose);
					//Update the lastPose sampled
					lastPose = newPose;
									}

							}
					}


		PoseWithInfo newPose;	
		newPose.pose = {path.poses.back().pose.position.x, path.poses.back().pose.position.y, 0.0};

		float distancePreviousPose = sqrt(pow((lastPose.pose[0] - newPose.pose[0]),2) + pow((lastPose.pose[1] - newPose.pose[1]),2));

		if (distancePreviousPose >= _lastSampleThreshold){ //This should be the xy_threshold set in the local planner
			bool nearToAborted = false;
			for (int j = 0; j < _abortedGoals.size(); j ++){
				float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose.pose[0]),2) + pow((_abortedGoals[j][1] - newPose.pose[1]),2));
				if (distanceAbortedGoal < _nearCentroidsThreshold){
					nearToAborted = true;
					break;
					}
				}
				
			if (!nearToAborted){
					
				float predictedAngle = predictAngle(lastPose.pose, newPose.pose);

				//Fill some fields of PoseWithInfo
				newPose.pose[2] = predictedAngle;
				newPose.predictedAngle = predictedAngle;
				newPose.index = path.poses.size() - 1;
				newPose.planLenght = path.poses.size() - 1;

				//Sample the NewPose
				vecSampledPoses.push_back(newPose);
				//Update the lastPose sampled
				lastPose = newPose;

								}
					
					}

	
	}


	return vecSampledPoses;

}







PoseWithInfo PathsRollout::extractBestPose(srrg_scan_matcher::Cloud2D cloud){
	
	PoseWithInfo goalPose;
	Isometry2f transform;
	Vector3f pose;
	Vector3f laserPose;

	for (int i = 0; i < _vectorSampledPoses.size(); i ++){

		pose[0] = _vectorSampledPoses[i].pose[0]; 
		pose[1] = _vectorSampledPoses[i].pose[1];

		bool isSamePosition = false;
		float distanceX = fabs(pose[0] - _tfMapToBase.getOrigin().x());
		float distanceY = fabs(pose[1] - _tfMapToBase.getOrigin().y());
		if ((distanceX < _xyThreshold) &&(distanceY < _xyThreshold)){
			isSamePosition = true;
		}


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;
			Rotation2D<float> rot(yawAngle);

			pose[2] = yawAngle;

			//If I'm considering a pose too close to the current one, discard rotations too similar to the current one.
			//Errors in the prediction function could make the robot to always remain in the same place.
			if (isSamePosition){
				Rotation2D<float> currentRot(tf::getYaw(_tfMapToBase.getRotation()));
				Rotation2D<float> differenceRot = currentRot.inverse()*rot;

				float distanceYaw = atan2(differenceRot.toRotationMatrix()(1,0), differenceRot.toRotationMatrix()(0,0));

				if (fabs(distanceYaw) < M_PI_2){
					continue;
				}
			}

			int countFrontier = computeVisiblePoints(pose, _laserOffset, cloud, _unknownCellsCloud->size());	


			float score = computePoseScore(_vectorSampledPoses[i], yawAngle, countFrontier);

			//std::cout<<i<<"-"<<j<<" "<<laserPose[0]<<" "<<laserPose[1]<<" "<<laserPose[2]<<" points: "<<countFrontier<<" score: "<<score <<std::endl;


			if (score > goalPose.score){
				//std::cout<<"GOAL: "<< pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" SCORE: "<<score<<std::endl;
				goalPose.pose = pose;
				goalPose.score = score;
				goalPose.predictedAngle = _vectorSampledPoses[i].pose[2]; 
			}

		}

		
	}
	
	std::cout<<"predictedAngle = "<< goalPose.predictedAngle<<std::endl;
	return goalPose;
}


int PathsRollout::computeVisiblePoints(Vector3f robotPose, Vector2f laserOffset,srrg_scan_matcher::Cloud2D cloud, int numInterestingPoints){

	int visiblePoints = 0;

	Vector3f laserPose;

	float yawAngle = robotPose[2];
	Rotation2D<float> rot(yawAngle);
	Vector2f laserOffsetRotated = rot*laserOffset;

	laserPose[0] = robotPose[0] + laserOffsetRotated[0];
	laserPose[1] = robotPose[1] + laserOffsetRotated[1];
	laserPose[2] = yawAngle;

	Isometry2f transform = v2t(laserPose);

	Isometry2f pointsToLaserTransform = transform.inverse();

	FloatVector _ranges;
	IntVector _pointsIndices;

	//_projector->sparseProjection(_ranges, _pointsIndices, pointsToLaserTransform, cloud);
	visiblePoints = _projector->areaProjection(pointsToLaserTransform, *_unknownCellsCloud, *_occupiedCellsCloud);
/* 
	cv::Mat testImage = cv::Mat(20/0.05, 20/0.05, CV_8UC1);
	testImage.setTo(cv::Scalar(0));
	cv::circle(testImage, cv::Point(robotPose[1]/0.05,robotPose[0]/0.05), 5, 200);
	cv::circle(testImage, cv::Point(laserPose[1]/0.05, laserPose[0]/0.05), 1, 200);
	std::stringstream title;
	title << "virtualscan_test/test_"<<_imageCount<<".jpg"; 
	_imageCount++;
	*/
	/*for (int k = 0; k < _pointsIndices.size(); k++){
		if (_pointsIndices[k] != -1){
			//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/0.05,cloud[_pointsIndices[k]].point()[1]/0.05) = 127;
			if (_pointsIndices[k] < numInterestingPoints){
				visiblePoints ++;
				//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/0.05,cloud[_pointsIndices[k]].point()[1]/0.05) = 255;

							}
						}
					}
	//cv::imwrite(title.str(),testImage);
*/
	return visiblePoints;

}



bool PathsRollout::isActionDone(MoveBaseClient *ac){

	actionlib::SimpleClientGoalState state = ac->getState();

	if ((state == actionlib::SimpleClientGoalState::ABORTED)||(state == actionlib::SimpleClientGoalState::SUCCEEDED)||(state == actionlib::SimpleClientGoalState::PREEMPTED)||(state == actionlib::SimpleClientGoalState::REJECTED)||(state == actionlib::SimpleClientGoalState::LOST))
    {
    	return true;
    }
    return false;

}


float PathsRollout::computePoseScore(PoseWithInfo pose, float orientation, int numVisiblePoints){


	float distanceFromStartWeight = 1.25;
	float distanceFromGoalWeight = - 0.1;
	float angleDistanceWeight = 0.15;

	float distanceFromStart = float(pose.index)/_longestPlan;
	float distanceFromGoal = 1.0 - float(pose.index)/pose.planLenght;

	Rotation2D<float> predictedRot(pose.predictedAngle);
	Rotation2D<float> desiredRot(orientation);

	float angleDifference = (desiredRot.inverse()*predictedRot).smallestAngle();

	float angularCost = 0.0;

	if (fabs(angleDifference) > _intervalOrientation){
		angularCost = fabs(angleDifference)/M_PI;	}
		
	float cost = distanceFromStartWeight * distanceFromStart + distanceFromGoalWeight * distanceFromGoal + angleDistanceWeight*angularCost;

	float decay = - cost * _lambda;
	
	float score = numVisiblePoints * exp(decay);

	std::cout<<_imageCount<<": index-> "<<pose.index<<"/"<<pose.planLenght <<" preferred angle-> "<<pose.predictedAngle<< " angle-> "<<orientation<<"("<<angleDifference<<") visiblePoints-> "<<numVisiblePoints<<" ---> cost:"<<cost<<" score: "<<score<<std::endl;

	return score;
}


float PathsRollout::predictAngle(Vector3f currentPose, Vector3f nextPosition){

	Vector2f previousVersor = {cos(currentPose[2]), sin(currentPose[2])};
	Vector2f newVersor = {nextPosition[0] - currentPose[0], nextPosition[1] - currentPose[1]};
	float predictedAngleDifference = acos(newVersor.normalized().dot(previousVersor.normalized()));



	Rotation2D<float> oldRot(currentPose[2]);
	Rotation2D<float> newRot(predictedAngleDifference);

	float predictedAngle = (oldRot*newRot).smallestPositiveAngle();

	return predictedAngle;

}





void PathsRollout::setAbortedGoals(Vector2fVector abortedGoals){
	_abortedGoals = abortedGoals;
}

void PathsRollout::setUnknownCellsCloud(Cloud2D* cloud){
	_unknownCellsCloud = cloud;
}

void PathsRollout::setOccupiedCellsCloud(Cloud2D* cloud){
	_occupiedCellsCloud = cloud;
}

void PathsRollout::setMapMetaData(nav_msgs::MapMetaData mapMetaDataMsg){
	_resolution = mapMetaDataMsg.resolution;
	_mapOriginX = mapMetaDataMsg.origin.position.x;
	_mapOriginY = mapMetaDataMsg.origin.position.y;
}
