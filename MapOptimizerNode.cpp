/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/MsgConversion.h"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread.hpp>

#include "rtabmap/core/Compression.h"
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace rtabmap;

class MapOptimizer
{

public:
	MapOptimizer() :
		mapFrameId_("map"),
		odomFrameId_("odom"),
		globalOptimization_(true),
		optimizeFromLastNode_(false),
		mapToOdom_(rtabmap::Transform::getIdentity()),         // returns 4x3 identity matrix
		transformThread_(0)
	{
		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		double epsilon = 0.0;
		bool robust = true;
		bool slam2d =false;
		int strategy = 1;                                      // 0=TORO, 1=g2o, 2=GTSAM            
		int iterations = 5;
		bool ignoreVariance = false;


		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);
		pnh.param("iterations", iterations, iterations);
		pnh.param("ignore_variance", ignoreVariance, ignoreVariance);
		pnh.param("global_optimization", globalOptimization_, globalOptimization_);
		pnh.param("optimize_from_last_node", optimizeFromLastNode_, optimizeFromLastNode_);
		pnh.param("epsilon", epsilon, epsilon);
		pnh.param("robust", robust, robust);
		pnh.param("slam_2d", slam2d, slam2d);
		pnh.param("strategy", strategy, strategy);


		UASSERT(iterations > 0);

		ParametersMap parameters;
		parameters.insert(ParametersPair(Parameters::kOptimizerStrategy(), uNumber2Str(strategy)));
		parameters.insert(ParametersPair(Parameters::kOptimizerEpsilon(), uNumber2Str(epsilon)));
		parameters.insert(ParametersPair(Parameters::kOptimizerIterations(), uNumber2Str(iterations)));
		parameters.insert(ParametersPair(Parameters::kOptimizerRobust(), uBool2Str(robust)));
		parameters.insert(ParametersPair(Parameters::kOptimizerSlam2D(), uBool2Str(slam2d)));
		parameters.insert(ParametersPair(Parameters::kOptimizerVarianceIgnored(), uBool2Str(ignoreVariance)));
		optimizer_ = Optimizer::create(parameters);

		double tfDelay = 0.05; // 20 Hz
		bool publishTf = true;
		pnh.param("publish_tf", publishTf, publishTf);
		pnh.param("tf_delay", tfDelay, tfDelay);

		mapDataTopic_ = nh.subscribe("rtabmap/mapData", 100, &MapOptimizer::mapDataReceivedCallback, this); // node /rtabmap/rtabmap=>CoreWrapper is publishing on this topic
		
		//Kamal
		//landmarkPoseTopic_ = nh.subscribe("rtabmap/landmark_pose", 100, &MapOptimizer::landmarkPoseReceivedCallback, this);

		//republish optimized mapdata and graph
		mapDataPub_ = nh.advertise<rtabmap_ros::MapData>(nh.resolveName("mapData")+"_optimized", 10);
		mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>(nh.resolveName("mapData")+"Graph_optimized", 10);

		//Dina
		landmarkPosePub_ = nh.advertise<visualization_msgs::Marker>("LandmarkOptimized", 1);
		robotPosePub_= nh.advertise<visualization_msgs::Marker>("RobotOptimized", 1);

		if(publishTf)
		{
			ROS_INFO("map_optimizer will publish tf between frames \"%s\" and \"%s\"", mapFrameId_.c_str(), odomFrameId_.c_str());
			ROS_INFO("map_optimizer: map_frame_id = %s", mapFrameId_.c_str());
			ROS_INFO("map_optimizer: odom_frame_id = %s", odomFrameId_.c_str());
			ROS_INFO("map_optimizer: tf_delay = %f", tfDelay);
			transformThread_ = new boost::thread(boost::bind(&MapOptimizer::publishLoop, this, tfDelay));
		}

		landmarks.open("../optimization.txt");
		landmarks<<"file initialized in MapOptimizer"<<std::endl;
	}

	~MapOptimizer()
	{
		if(transformThread_)
		{
			transformThread_->join();
			delete transformThread_;
		}
	}

	void publishLoop(double tfDelay)
	{
		if(tfDelay == 0)
			return;
		ros::Rate r(1.0 / tfDelay);      // 1.0/0.05 = 20 Hz
		while(ros::ok())
		{
			mapToOdomMutex_.lock();
			ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfDelay);
			geometry_msgs::TransformStamped msg;   
			msg.child_frame_id = odomFrameId_;
			msg.header.frame_id = mapFrameId_;
			msg.header.stamp = tfExpiration;
			rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);      // convert transform matrix to geometry_msgs
			tfBroadcaster_.sendTransform(msg);
			mapToOdomMutex_.unlock();
			r.sleep();
		}
	}

	/*Dina Youakim*/
	void initializeLandmarkCounter(rtabmap_ros::MapDataConstPtr & msg)         // msg => modifiedMapData (rtabmap_ros/MapData.msg) 
	{
		int maxNodeID = -1;
		for(unsigned int i=0; i < msg->nodes.size(); ++i)                      
		{
			rtabmap_ros::NodeData currentNode = msg->nodes[i];
			if(maxNodeID < currentNode.id)
				maxNodeID = currentNode.id;
		}
		landmarkCounter = maxNodeID + 1;
		//landmarkCounter = maxNodeID;
	}

	/* Dina Youakim */
	void fillOldLandmarks(rtabmap_ros::MapDataConstPtr & msg,std::map<int, rtabmap::Transform> &posesOut, std::multimap<int, rtabmap::Link> &constraints)
	{
		
		if(!landmarksCache_.empty())
		{
			landmarks<<"start inserting old landmark nodes "<<std::endl;
			bool firstLandmarkObseervation = true;
			for(std::map<int, rtabmap::Transform>::iterator iter = landmarksCache_.begin(); iter != landmarksCache_.end(); ++iter)
			{
				//Landmark pose in base_link frame
				rtabmap::Transform landmarkObservation = iter->second;
				rtabmap::Transform landmarkToRobotPose;
				for (unsigned int i=0; i<msg->nodes.size(); ++i)
				{
					if(msg->nodes[i].id == iter->first)
					{
						double roll, pitch, yaw;

						tf::Quaternion quat;
			    		tf::quaternionMsgToTF(msg->nodes[i].pose.orientation, quat);
						tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

						//Get the landmark pose in the odom frame of reference to be added to the graph
						rtabmap::Transform robotPose(msg->nodes[i].pose.position.x,msg->nodes[i].pose.position.y,msg->nodes[i].pose.position.z,roll,pitch,yaw);
						
						landmarkToRobotPose = (robotPose)*landmarkObservation;

						if(firstLandmarkObseervation)
							posesOut.insert(std::make_pair(landmarkCounter,landmarkObservation));
						firstLandmarkObseervation = false;
						break;
					}
				}

				/* Create landmark link*/
				rtabmap::Link link;
				link.setFrom(iter->first);
				link.setTo (landmarkCounter);
				link.setType (rtabmap::Link::kLandmark);
				link.setTransform(landmarkToRobotPose);
				constraints.insert(std::make_pair(link.from(),link));

				landmarks << "old landmarks filled" << std::endl;


				landmarks<<"landmark node created between "<<iter->first<<","<<landmarkCounter<<std::endl;

				landmarks<<"the landmark pose is: "<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()
						 <<","<<landmarkObservation.theta()<<std::endl;

				landmarks<<"the landmark constraint is: "<<landmarkToRobotPose.translation().x()<<","<<landmarkToRobotPose.translation().y()
				         <<","<<landmarkToRobotPose.translation().theta()<<std::endl;
			}

			ROS_INFO_STREAM("Old Landmarks filled");
		}

	}

	/* Kamal: add new landmark link to constraint rather than linksOut */
    void createLandmarks(rtabmap_ros::MapDataConstPtr & msg, std::map<int, rtabmap::Transform> &poses, std::multimap<int, Link> &constraints)
	{
		for(unsigned int i=0; i<msg->nodes.size(); ++i)
		{
			rtabmap_ros::NodeData currentNode = msg->nodes[i];
			
			if(!currentNode.userData.empty())
			{
				cv::Mat landmark = cv::Mat::zeros(2,3, CV_32F);

				landmark = rtabmap::uncompressData(currentNode.userData);

				rtabmap::Transform landmarkObservation(landmark.at<float>(0,0),landmark.at<float>(0,1),landmark.at<float>(0,2),
					                                   landmark.at<float>(1,0),landmark.at<float>(1,1),landmark.at<float>(1,2));

				double roll, pitch, yaw;
				tf::Quaternion quat;
    			tf::quaternionMsgToTF(currentNode.pose.orientation, quat);
				tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

				rtabmap::Transform robotPose(currentNode.pose.position.x, currentNode.pose.position.y, currentNode.pose.position.z, roll, pitch, yaw);
				rtabmap::Transform landmarkToRobotPose =  (robotPose)*landmarkObservation;
				
				//landmarks<<"observed at time: "<<currentNode.stamp<<", for node: "<<currentNode.id<<std::endl;

				float r,p,y;
				landmarkObservation.getEulerAngles(r,p,y);

				landmarks << "landmark counter: " << landmarkCounter << std::endl;
				
				//landmarks<<"landmark node created between "<<currentNode.id<<","<<landmarkCounter<<std::endl;
				//landmarks<<"the landmark pose is: "<<landmarkObservation.translation().x()<<","<<landmarkObservation.translation().y()<<","<<landmarkObservation.theta()<<std::endl;
			    //landmarks<<"the landmark constraint is: "<<landmarkToRobotPose.translation().x()<<","<<landmarkToRobotPose.translation().y()<<","<<landmarkToRobotPose.theta()<<std::endl;*

				//landmarks<< "landmark observed at: x:" << landmarkObservation.translation().x()<<", y:"<<landmarkObservation.translation().y()<<", theta:"<<landmarkObservation.theta()<<std::endl;
				
				//landmarks<< "landmark observed at: x:" << landmarkObservation.x()<<", y:"<<landmarkObservation.y()<<", theta:"<<landmarkObservation.theta()<<std::endl;

				/* Create landmark link*/
				rtabmap::Link link;
				link.setFrom(currentNode.id);
				link.setTo(landmarkCounter);
				link.setType(rtabmap::Link::kLandmark);
				link.setTransform(landmarkToRobotPose);
				//link.setInfMatrix() ???;
				//link.setVariance() ???;
				constraints.insert(std::make_pair(link.from(),link));
				landmarks << "new landmark link created from "<< currentNode.id << " to " << landmarkCounter << std::endl;


				//std::map<int,std::vector< rtabmap::Transform > >::iterator  it;

				//if the cache is empty; it means it is first time to observe the landmark
				if(landmarksCache_.empty())
				{
					poses.insert(std::make_pair(landmarkCounter,landmarkObservation));
					//landmarksCache_.insert(std::make_pair<int, rtabmap::Transform>(currentNode.id,landmarkObservation));
				}
					
				landmarksCache_.insert(std::make_pair<int, rtabmap::Transform>(currentNode.id,landmarkObservation));				

				ROS_INFO_STREAM("New Landmarks created");
				
			}
			
		}

		landmarks<<"current landmark cache size is: "<<landmarksCache_.size()<<std::endl;

	}

	

    /* Kamal: landmark pose received from core_wrapper */    // => received in userData in robot_frame
	/*void landmarkPoseReceivedCallback(const geometry_msgs::PoseWithCovarianceStamped & pose)
	{
		tf::Quaternion q (pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		landmarks << "landmark pose in camera_landmark: stamp:" << pose.header.stamp 
		          << ", x:" << pose.pose.pose.position.x << ", y:" << pose.pose.pose.position.y 
		          << ", z:" << pose.pose.pose.position.z << ", roll:" << roll << ", pitch:" << pitch << ", yaw:" << yaw << std::endl;;
		
	}*/


	void mapDataReceivedCallback(const rtabmap_ros::MapDataConstPtr & msg)
	{
		// save new poses and constraints
		// Assuming that nodes/constraints are all linked together
		UASSERT(msg->graph.posesId.size() == msg->graph.poses.size());

		bool dataChanged = false;

		//Dina Youakim
		rtabmap_ros::MapDataConstPtr modifiedMapData = msg; 
		landmarks << std::endl;
		landmarks << "new map data received at:" << modifiedMapData->header.stamp << std::endl;
		landmarks << "modifiedMapData->graph.links.size(): " << modifiedMapData->graph.links.size() << std::endl;

		for(unsigned int i = 0; i < modifiedMapData->nodes.size(); ++i)
		{
			rtabmap_ros::NodeData currentNode = msg->nodes[i];

			if(!currentNode.userData.empty())
			{
				landmarks << "user data not empty" << std::endl;
				cv::Mat userData =  cv::Mat::zeros(2,3, CV_32F);
				userData = rtabmap::uncompressData(currentNode.userData);
				for(int i = 0; i < userData.rows; i++)
				{
					for(int j = 0; j < userData.cols; j++)
					{
						landmarks << "userData("<<i<<","<<j<<"):"<<userData.at<float>(i,j)<<std::endl;
					}
				}                    

			}                              
				
			else
				landmarks << "user data empty" << std::endl;
		}


		

		std::multimap<int, Link> newConstraints;
		for(unsigned int i = 0; i < modifiedMapData->graph.links.size(); ++i)
		{
			Link link = rtabmap_ros::linkFromROS(modifiedMapData->graph.links[i]);
			newConstraints.insert(std::make_pair(link.from(), link));			

			bool edgeAlreadyAdded = false;
			for(std::multimap<int, Link>::iterator iter = cachedConstraints_.lower_bound(link.from());
					iter != cachedConstraints_.end() && iter->first == link.from();
					++iter)
			{
				if(iter->second.to() == link.to())
				{
					edgeAlreadyAdded = true;
					if(iter->second.transform().getDistanceSquared(link.transform()) > 0.0001)
					{
						ROS_WARN("%d ->%d (%s vs %s)",iter->second.from(), iter->second.to(), iter->second.transform().prettyPrint().c_str(),
								link.transform().prettyPrint().c_str());
						dataChanged = true;
						landmarks << "data changed 1" << std::endl;
					}
					
				} 
			}

			if(!edgeAlreadyAdded)
			{
				cachedConstraints_.insert(std::make_pair(link.from(), link));
				landmarks<<"new constraint added from node "<<link.from()<<" to node "<<link.to()<<std::endl; 
			}

		}

		for(std::multimap<int, Link>::iterator iter = newConstraints.begin(); iter != newConstraints.end(); ++iter)
		{
			landmarks << "newConstraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
		}

		landmarks << "modifiedMapData->nodes.size():" << modifiedMapData->nodes.size() << std::endl;
		// add new odometry poses
		std::map<int, Signature> newNodeInfos;     //save signature for each node
		for(unsigned int i=0; i < modifiedMapData->nodes.size(); ++i)
	    {
			int id = modifiedMapData->nodes[i].id;
			Transform pose = rtabmap_ros::transformFromPoseMsg(modifiedMapData->nodes[i].pose);//3D affine transformation from geometry_msgs/Pose; gives the robot_pose
			
			//Signature s = rtabmap_ros::nodeInfoFromROS(modifiedMapData->nodes[i]); /* dont use nodeInfoFromROS() because it 
			                                                                         /* doesn't insert the userData in the signature. */

			Signature s = rtabmap_ros::nodeDataFromROS(modifiedMapData->nodes[i]);   /* nodeDataFromROS() inserts the userData as well */
			
			newNodeInfos.insert(std::make_pair(id, s));

			std::pair<std::map<int, Signature>::iterator, bool> p = cachedNodeInfos_.insert(std::make_pair(id, s));
			if(!p.second && pose.getDistanceSquared(cachedNodeInfos_.at(id).getPose()) > 0.0001)
			{
				dataChanged = true;
				landmarks << "data changed 2" << std::endl;
			}


			landmarks<<"robot_pose: frame_id: "<<modifiedMapData->header.frame_id<<", nodeId: "<<s.id()<<", pose.x(): "<<pose.x()
                     <<", pose.y(): "<<pose.y()<<", pose.z(): "<<pose.z()<<", pose.theta(): "<<pose.theta()<<std::endl;       // robot poses
		
		}


		// for error
		for(std::map<int, Signature>::iterator it=newNodeInfos.begin(); it!=newNodeInfos.end();++it)
		{

			if(newNodeInfos.size()>1 && modifiedMapData->graph.links.size()==0)
			{
				newNodeInfos.erase(newNodeInfos.end());
				landmarks << "node erased" << std::endl;
			}

			landmarks << "newNodeInfos: id:"<<it->first<<", x:"<<it->second.getPose().x()<<", y:" <<it->second.getPose().y()<<", z:"<<it->second.getPose().z()<<", theta:"<<it->second.getPose().theta()
					  << std::endl; 
		}

		if(dataChanged)
		{
			ROS_WARN("Graph data has changed! Reset cache...");
			cachedConstraints_ = newConstraints;
			cachedNodeInfos_ = newNodeInfos;
		}
		for(std::multimap<int, Link>::iterator iter = cachedConstraints_.begin(); iter != cachedConstraints_.end(); ++iter)
		{
			landmarks << "cachedConstraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
		}

		//match poses in the graph
		std::multimap<int, Link> constraints;
		std::map<int, Signature> nodeInfos;
		
		if(globalOptimization_)
		{
			constraints = cachedConstraints_;
			nodeInfos = cachedNodeInfos_;
		}
		else
		{
			constraints = newConstraints;
			for(unsigned int i=0; i<modifiedMapData->graph.posesId.size(); ++i)
			{
				std::map<int, Signature>::iterator iter = cachedNodeInfos_.find(modifiedMapData->graph.posesId[i]);
				if(iter != cachedNodeInfos_.end())
				{
					nodeInfos.insert(*iter);
				}
				else
				{
					ROS_ERROR("Odometry pose of node %d not found in cache!", modifiedMapData->graph.posesId[i]);
					return;
				}
			}
		}

		for(std::multimap<int, Link>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
		{
			landmarks << "constraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
		}


		std::map<int, Transform> poses;
		for(std::map<int, Signature>::iterator iter=nodeInfos.begin(); iter!=nodeInfos.end(); ++iter)
		{
			poses.insert(std::make_pair(iter->first, iter->second.getPose()));
			/*landmarks << "id:" << iter->first << std::endl;
			

			cv::Mat userData =  cv::Mat::zeros(2,3, CV_32F);
			userData = rtabmap::uncompressData(iter->second.sensorData().userDataCompressed());
			for(int i = 0; i < userData.rows; i++)
			{
				for(int j = 0; j < userData.cols; j++)
				{
					landmarks << "sensorData.userData("<<i<<","<<j<<"):"<<userData.at<float>(i,j)<<std::endl;
				}
			}

			if(!iter->second.sensorData().userDataCompressed().empty())
			{
				landmarks << "sensorData.userdata is not empty" << std::endl;
				cv::Mat landmark = cv::Mat::zeros(2,3, CV_32F);

				landmark = rtabmap::uncompressData(iter->second.sensorData().userDataCompressed());

				rtabmap::Transform landmarkTransform(landmark.at<float>(0,0),landmark.at<float>(0,1),landmark.at<float>(0,2),
					                                   landmark.at<float>(1,0),landmark.at<float>(1,1),landmark.at<float>(1,2));
					
				if(poses.find(iter->second.id()) == poses.end())                            // pose not already in map 
				{
					rtabmap::Link link;
					link.setFrom(iter->first);
					link.setTo(iter->second.id());
					link.setType(rtabmap::Link::kLandmark);
					link.setTransform(landmarkTransform);
					//link.setInfMatrix() ???;
					//link.setVariance() ???;
					constraints.insert(std::make_pair(link.from(),link));
				}

			}
			else
				landmarks << "sensorData.userdata is empty" << std::endl;*/


		}

		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			landmarks << "Pose: id:" << iter->first << std::endl;
		}

		Transform landmarkPose, currentRobotPose;

		// Optimize only if there is a subscriber
		if(mapDataPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers())
		{
			landmarks<<"Optimization enabled!!!!!"<<std::endl;
			UTimer timer;
			std::map<int, Transform> optimizedPoses, landmarkOptimizedPoses, robotOptimizedPoses;
			Transform mapCorrection = Transform::getIdentity();
			Transform landmarkCorrection = Transform::getIdentity();
			std::map<int, rtabmap::Transform> posesOut, posesOutNoLandmarks, posesOutLandmarks;
			std::multimap<int, rtabmap::Link> linksOut;



			if(poses.size() > 1 && constraints.size() > 0)
			{
				std::cout << "In first if condition" << std::endl;
				
				int fromId = optimizeFromLastNode_ ? poses.rbegin()->first : poses.begin()->first;

				
				//Dina Youakim: since landmark nodes IDs are dynamic and change everytime given the max ID we have in the robot pose nodes
				//we need to compute this max and start creating landmark nodes from max+1
				initializeLandmarkCounter(modifiedMapData);
				ROS_INFO_STREAM("counter initialized");




				//Dina Youakim: before adding new landmarks; process old ones from the cache and add them to poses and links with new IDs 
				//given the max used IDs for the normal nodes
				fillOldLandmarks(modifiedMapData, posesOutLandmarks, constraints);
				landmarks << "after fillOldLandmarks: posesOut.size(): " << posesOut.size() << ", linksOut.size(): " << linksOut.size() << ", constraints.size(): "<< constraints.size() 
						  << ", posesOutLandmarks.size(): " << posesOutLandmarks.size()  
						  << ", posesOutNoLandmarks.size(): " << posesOutNoLandmarks.size() << std::endl;
				for(std::multimap<int, rtabmap::Link>::iterator iter = linksOut.begin(); iter != linksOut.end(); ++iter)
				{
					landmarks << "after fillOldLandmarks: linksOut between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
				for(std::multimap<int, Link>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					landmarks << "after fillOldLandmarks: constraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutLandmarks.begin(); iter != posesOutLandmarks.end(); ++iter)
				{
					landmarks << "after fillOldLandmarks: posesOutLandmarks: x:" << iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl; 
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutNoLandmarks.begin(); iter != posesOutNoLandmarks.end(); ++iter)
				{
					landmarks << " posesOutNoLandmarks: x:" << iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl; 
				}




				//Dina Youakim create new landmarks here add to poses and links
				createLandmarks(modifiedMapData, posesOutLandmarks, constraints);
				landmarks << "after createLandmarks: posesOut.size(): " << posesOut.size() << ", linksOut.size(): " << linksOut.size() << ", constraints.size(): " << constraints.size() 
						  << ", posesOutLandmarks.size(): " << posesOutLandmarks.size() << ", posesOutNoLandmarks.size(): " << posesOutNoLandmarks.size() << std::endl;
				for(std::multimap<int, rtabmap::Link>::iterator iter = linksOut.begin(); iter != linksOut.end(); ++iter)
				{
					landmarks << "linksOut: between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
				for(std::multimap<int, Link>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					landmarks << "constraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutLandmarks.begin(); iter != posesOutLandmarks.end(); ++iter)
				{
					landmarks << "posesOutLandmarks: id:" << iter->first << ", x: "<< iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl; 
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutNoLandmarks.begin(); iter != posesOutNoLandmarks.end(); ++iter)
				{
					landmarks << "posesOutNoLandmarks: id:" << iter->first << ", x: "<< iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl;  
				}


				posesOutNoLandmarks = poses;
				poses.insert(posesOutLandmarks.begin(), posesOutLandmarks.end());


				landmarks << "before computing connected graph: poses.size():" << poses.size() << ", constraints.size(): " << constraints.size() << std::endl;
				optimizer_->getConnectedGraph(fromId, poses, constraints, posesOut, linksOut);
                ROS_INFO_STREAM("connected graph computed");
                landmarks << "connected graph computed: posesOut.size(): " << posesOut.size()
                 		  << ", linksOut.size(): " << linksOut.size() 
                 		  << ", constraints.size(): "<< constraints.size() 
                 		  << ", posesOutLandmarks.size(): " << posesOutLandmarks.size()  
						  << ", posesOutNoLandmarks.size(): " << posesOutNoLandmarks.size() << std::endl;
                for(std::multimap<int, rtabmap::Link>::iterator iter = linksOut.begin(); iter != linksOut.end(); ++iter)
				{
					landmarks << "linksOut: between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
                for(std::multimap<int, Link>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					landmarks << "constraint between " << iter->second.from() << " to " << iter->second.to() << std::endl; 
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutLandmarks.begin(); iter != posesOutLandmarks.end(); ++iter)
				{
					landmarks << "posesOutLandmarks: id:" << iter->first << ", x: "<< iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl;  
				}
				for(std::map<int, rtabmap::Transform>::iterator iter = posesOutNoLandmarks.begin(); iter != posesOutNoLandmarks.end(); ++iter)
				{
					landmarks << "posesOutNoLandmarks: id:" << iter->first << ", x: "<< iter->second.x()<<", y:" << iter->second.y() << ", theta:" << iter->second.theta() << std::endl;  
				}
				for (std::multimap<int, rtabmap::Link>::iterator iter = linksOut.begin(); iter != linksOut.end(); ++iter)
				{
					landmarks << "linksOut: link.from(): " << iter->second.from() << ", link.to(): " << iter->second.to() << std::endl;
				}

				//optimizedPoses = optimizer_->optimizeWithLandmarks(fromId, posesOut, linksOut);   // optimizedPoses is of type std::map<int, Transform>
				optimizedPoses = optimizer_->optimize(fromId, posesOut, linksOut);   // optimizedPoses is of type std::map<int, Transform>
				
				for (std::map<int, Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
				{
					landmarks << "optimizedPoses: id:" << iter->first << ", x:" << iter->second.x() << ", y:" << iter->second.y()
							  << ", theta:" << iter->second.theta() << std::endl;

					landmarks << "landmarkCounter: " << landmarkCounter << std::endl;

					if(iter->first == landmarkCounter)
					{
						landmarkPose = iter->second;
						//landmarkPose = optimizedPoses.at(posesOutLandmarks.begin()->first);
						//landmarks<<"last landmark pose chosen: "<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta()<<std::endl;
						break;
					}
				}

				mapToOdomMutex_.lock();

				currentRobotPose = optimizedPoses.at(posesOutNoLandmarks.rbegin()->first);
				landmarks << "currentRobotPose: x:" << currentRobotPose.x() << ", y:" << currentRobotPose.y()
						  << ", theta:" << currentRobotPose.theta() << std::endl;

				mapCorrection = currentRobotPose * posesOutNoLandmarks.rbegin()->second.inverse();
				//landmarkCorrection = optimizedPoses.at(posesOutLandmarks.rbegin()->first) * posesOutLandmarks.rbegin()->second.inverse();
				//landmarkPose.x() -= landmarkCorrection.x();
				//landmarkPose.y() -= landmarkCorrection.y();
				//landmarkPose.z() -= landmarkCorrection.z();
				landmarks<<"mapCorrection: x:"<<mapCorrection.x()<<", y:"<<mapCorrection.y()<<", z:"<<mapCorrection.z()<<", theta:"<<mapCorrection.theta()<<std::endl;
				//landmarks<<"Landmark correction: "<<landmarkCorrection.x()<<","<<landmarkCorrection.y()<<","<<landmarkCorrection.z()<<","<<landmarkCorrection.theta()<<std::endl;
				mapToOdom_ = mapCorrection;
				mapToOdomMutex_.unlock();
			}



			else if(poses.size() == 1 && constraints.size() == 0)
			{
				std::cout << "In second if condition" << std::endl;

				optimizedPoses = poses;
				for(std::map<int,rtabmap::Transform>::iterator it = optimizedPoses.begin(); it != optimizedPoses.end(); ++it)
				{
					landmarks << "optimizedPoses: id:" << it->first << ", x:" << it->second.x() << ", y:" << it->second.y()
							  << ", theta:" << it->second.theta() << std::endl;
				}
				
			}



			else if(poses.size() || constraints.size())
			{
				std::cout << "In third if condition" << std::endl;

				ROS_ERROR("map_optimizer: Poses=%d and edges=%d (poses must "
					   "not be null if there are edges, and edges must be null if poses <= 1)",
					  (int)poses.size(), (int)constraints.size());
			}
            


			rtabmap_ros::MapData outputDataMsg;
			rtabmap_ros::MapGraph outputGraphMsg;
			rtabmap_ros::mapGraphToROS(optimizedPoses,
					linksOut,
					mapCorrection,
					outputGraphMsg);

			if(mapGraphPub_.getNumSubscribers())
			{
				outputGraphMsg.header = modifiedMapData->header;
				mapGraphPub_.publish(outputGraphMsg);
			}

			if(mapDataPub_.getNumSubscribers())
			{
				outputDataMsg.header = modifiedMapData->header;
				outputDataMsg.graph = outputGraphMsg;
				outputDataMsg.nodes = modifiedMapData->nodes;
				if(posesOutNoLandmarks.size() > modifiedMapData->nodes.size())
				{
					std::set<int> addedNodes;
					for(unsigned int i=0; i<modifiedMapData->nodes.size(); ++i)
					{
						addedNodes.insert(modifiedMapData->nodes[i].id);
					}
					std::list<int> toAdd;
					for(std::map<int, Transform>::iterator iter=posesOut.begin(); iter!=posesOut.end(); ++iter)
					{
						if(addedNodes.find(iter->first) == addedNodes.end())
						{
							toAdd.push_back(iter->first);
						}
					}
					if(toAdd.size())
					{
						int oi = outputDataMsg.nodes.size();
						outputDataMsg.nodes.resize(outputDataMsg.nodes.size()+toAdd.size());
						for(std::list<int>::iterator iter=toAdd.begin(); iter!=toAdd.end(); ++iter)
						{
							UASSERT(cachedNodeInfos_.find(*iter) != cachedNodeInfos_.end());
							rtabmap_ros::nodeDataToROS(cachedNodeInfos_.at(*iter), outputDataMsg.nodes[oi]);
							++oi;
						}
					}
				}
				mapDataPub_.publish(outputDataMsg);
			}

			
			/*geometry_msgs::PoseStamped landmarkPoseStamped;
			landmarkPoseStamped.header.stamp = ros::Time::now();
			rtabmap_ros::transformToPoseMsg(landmarkPose,landmarkPoseStamped.pose);    	
    		geometry_msgs::PoseStamped out;
    		out.header.stamp = landmarkPoseStamped.header.stamp;
    		out.header.frame_id = "odom";
    		if(tfListener_.waitForTransform("base_link", "odom",landmarkPoseStamped.header.stamp, ros::Duration(0.1))){
        		tfListener_.transformPose("odom",landmarkPoseStamped,out); 
        		
        	}*/
        	//Transform landmarkTransformToOdom = rtabmap_ros::transformFromPoseMsg(out.pose);

			//landmarkPose = mapCorrection*landmarkPose;
			
			/*visualization_msgs::Marker marker;
			marker.header.frame_id = mapFrameId_;
			marker.header.stamp = ros::Time::now();
			marker.ns = "landmark";
			marker.id = landmarkCounter;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = landmarkPose.x();
			marker.pose.position.y = landmarkPose.y();
			marker.pose.position.z = landmarkPose.z();
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(landmarkPose.theta());
			marker.pose.orientation = q;
			
			marker.scale.x = 0.08;
			marker.scale.y = 0.08;
			marker.scale.z = 0.08;
			marker.color.a = 1;
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;

			marker.type = visualization_msgs::Marker::CUBE;
			landmarkPosePub_.publish(marker);

			
			visualization_msgs::Marker marker2;
			marker2.header.frame_id = mapFrameId_;
			marker2.header.stamp = ros::Time::now();
			marker2.ns = "robot";
			marker2.id = landmarkCounter;
			marker2.action = visualization_msgs::Marker::ADD;
			marker2.pose.position.x = currentRobotPose.x();
			marker2.pose.position.y = currentRobotPose.y();
			marker2.pose.position.z = currentRobotPose.z();
			geometry_msgs::Quaternion q2 = tf::createQuaternionMsgFromYaw(currentRobotPose.theta());
			marker2.pose.orientation = q2;
			
			marker2.scale.x = 0.08;
			marker2.scale.y = 0.08;
			marker2.scale.z = 0.08;
			marker2.color.a = 1;
			marker2.color.r = 0.0;
			marker2.color.g = 1.0;
			marker2.color.b = 0.0;

			marker2.type = visualization_msgs::Marker::CUBE;
			robotPosePub_.publish(marker2);*/



			geometry_msgs::TransformStamped msg;
			msg.child_frame_id = "landmark";
			msg.header.frame_id = mapFrameId_;     // mapframeId
			msg.header.stamp = ros::Time::now() ;
			rtabmap_ros::transformToGeometryMsg(landmarkPose, msg.transform);
			tfBroadcaster_.sendTransform(msg);

			ROS_INFO_STREAM("Marker published with landmark pose "<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta());
			//landmarks<<"Marker published with pose "<<landmarkPose.x()<<","<<landmarkPose.y()<<","<<landmarkPose.theta()<<std::endl;
			landmarks <<"landmarkPose.x():"<<landmarkPose.x()<<", landmarkPose.y():"<<landmarkPose.y()<<", landmarkPose.z():"<<landmarkPose.z()<<", landmarkPose.theta():"<<landmarkPose.theta()<<std::endl;
			ROS_INFO("Time graph optimization = %f s", timer.ticks());
		}
	}

private:
	std::string mapFrameId_;
	std::string odomFrameId_;
	bool globalOptimization_;
	bool optimizeFromLastNode_;
	Optimizer * optimizer_;

	rtabmap::Transform mapToOdom_;
	boost::mutex mapToOdomMutex_;

	ros::Subscriber mapDataTopic_;

	//Kamal
	ros::Subscriber landmarkPoseTopic_;

	ros::Publisher mapDataPub_;
	ros::Publisher mapGraphPub_;
	ros::Publisher landmarkPosePub_, robotPosePub_;

	std::multimap<int, Link> cachedConstraints_;
	std::map<int, Signature> cachedNodeInfos_;
	//Dina Youakim: add this map to act like a chache for landmarks observed from previous times
	//As the landmarks are not saved in rtabmap_ros memory; they need to be saved here for next optimization cycles
	//The ID of a landmark is not so representative as it will change every cycle (could not use -ve IDs so every time ze compute the max of nodes IDs and start assigining to
	//landmark starting from max + 1). So the key of the map is the ID of the node to which it is attached and the value is the corrseponding landmark observation 
	std::map<int, rtabmap::Transform> landmarksCache_;

	tf2_ros::TransformBroadcaster tfBroadcaster_;
	tf::TransformListener tfListener_;

	boost::thread* transformThread_;
	std::ofstream landmarks;
	int landmarkCounter;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_optimizer");
	MapOptimizer optimizer;
	ros::spin();
	return 0;
}
