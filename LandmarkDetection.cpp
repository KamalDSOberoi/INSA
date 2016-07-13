/*
Author Dina Youakim
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "OPTM_Module_Component.h"
#include "landmarks_detection/GetLandmarkPose.h"
#include "landmarks_detection/GetRobotToLandmarkPose.h"
#include "landmarks_detection/LandmarkPose.h"

#include "landmark_detection.h"
//#include "featureMatching.h"

landmarkDetection detect_landmark;
//featureMatching match_features;

class LandmarkDetection
{
private:
        ros::NodeHandle nh;
        ros::Publisher odomPub_,robotPosePub_;
        ros::Subscriber robotStatusSub_, robotLocolizationSub_;
        ros::ServiceServer getLandmarkPoseSrv_,getRobotPoseSrv_;
        //integration with optimeter module
        OPTM_Module_Data::data data; // The data container
        OPTM_Module_Component component; // The module component which gets the data from the FBR module
        std::string lastUpdateTime;
        std::ofstream file;
        landmarks_detection::LandmarkPose updateLandmarkPose;

public:
    LandmarkDetection()
    {
        // Initialize the component with the IP address and port of the embedded NUC computer on the network
        component.initialize("192.168.0.65", 2100);
        ROS_INFO_STREAM("in landmark detection");
        //odomPub_ = nh.advertise<nav_msgs::Odometry>("localize_odom_in", 10);
        //robotPosePub_ = nh.advertise<nav_msgs::Odometry>("robot_to_landmark", 10);
        
        getLandmarkPoseSrv_= nh.advertiseService("/getLandmarkPose",&LandmarkDetection::GetLandmarkPose,this);   // provides landmark pose in camera frame
        getRobotPoseSrv_= nh.advertiseService("/getRobotPose",&LandmarkDetection::GetRobotToLandmarkPose,this);  // provides robot pose in landmark frame 
        lastUpdateTime = "";
        file.open("../landmarks.txt");
        file << "file initialized in LandmarkDetection" << std::endl; 
     /* while(true)
      {
        //ROS_INFO_STREAM("before sleeping");
        sleep(1);
        component.updateData(data);
        //ROS_INFO_STREAM("DATA RECEIED: ");
        //ROS_INFO_STREAM(data.strTimeStamp<<","<<data.adCameraCoordinates[0]<<","<<data.adCameraCoordinates[1]<<","<<data.adCameraCoordinates[2]);
        std::cout<<ros::Time::now()<<","<<data.adCameraCoordinates[0]<<","<<data.adCameraCoordinates[1]<<","<<data.adCameraCoordinates[2]<<std::endl;
        }*/
    }

    OPTM_Module_Component getComponent()
    {
      return component;
    }

    landmarks_detection::LandmarkPose getLandmarkUpdate()
    {
      return updateLandmarkPose;
    }

    bool computeLandmarkPose()
    {
      component.updateData(data);
      //std::cout<<data.strTimeStamp<<","<<data.adCameraCoordinates[0]<<","<<data.adCameraCoordinates[1]<<","<<data.adCameraCoordinates[2]<<std::endl;
      
      if(newLandmarkObservationReceived(data.strTimeStamp))
      {

        lastUpdateTime = data.strTimeStamp;

        /* rotation matrix */
        Eigen::Matrix3d landmarkToCameraRotation;
        landmarkToCameraRotation(0,0) = data.adFrameAttitude[0][0];
        landmarkToCameraRotation(0,1) = data.adFrameAttitude[0][1];
        landmarkToCameraRotation(0,2) = data.adFrameAttitude[0][2];
        landmarkToCameraRotation(1,0) = data.adFrameAttitude[1][0];
        landmarkToCameraRotation(1,1) = data.adFrameAttitude[1][1];
        landmarkToCameraRotation(1,2) = data.adFrameAttitude[1][2];
        landmarkToCameraRotation(2,0) = data.adFrameAttitude[2][0];
        landmarkToCameraRotation(2,1) = data.adFrameAttitude[2][1];
        landmarkToCameraRotation(2,2) = data.adFrameAttitude[2][2];

        Eigen::Vector3d ea = landmarkToCameraRotation.eulerAngles(0,1,2);
        //file<<"roll,pitch,yaw: "<<ea[0]<<", "<<ea[1]<<", "<<ea[2]<<std::endl;



        /* correction rotation matrix */
        Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(3.14, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
        Eigen::Quaternion<double> qt = yawAngle * pitchAngle * rollAngle;
        Eigen::Matrix3d rotationMatrix = qt.matrix();

        Eigen::Matrix3d landmarkToCameraRotationCorrected = landmarkToCameraRotation * rotationMatrix;

        Eigen::Vector3d ea1 = landmarkToCameraRotationCorrected.eulerAngles(0,1,2);
        //file<<"roll,pitch,yaw corrected: "<<ea1[0]<<", "<<ea1[1]<<", "<<ea1[2]<<std::endl;



        /* corrected rotation transform */
        Eigen::Transform<double, 3, Eigen::Affine> r;
        r = landmarkToCameraRotationCorrected; 



        /* translation transform*/ 
        Eigen::Translation<double, 3> landmarkToCameraTranslation(data.adCameraCoordinates[0], data.adCameraCoordinates[1], data.adCameraCoordinates[2]);
        Eigen::Transform<double, 3, Eigen::Affine> t;
        t = (landmarkToCameraTranslation);



        /* transformation */ 
        Eigen::Transform<double, 3, Eigen::Affine> landmarkToCameraTransform = r * t;
        Eigen::Transform<double, 3, Eigen::Affine> cameraToLandmarkTransform = landmarkToCameraTransform.inverse();

        //Eigen::Quaternionf q1(landmarkToCameraTransform.rotation());
        //Eigen::Matrix3f mat = q1.matrix();
        //Eigen::Vector3f ea1 = mat.eulerAngles(0,1,2);
        //file<<"roll, pitch, yaw from transformation:"<<ea1[0]<<", "<<ea1[1]<<", "<<ea1[2]<<std::endl;


        /* Pose Msg */ 
        geometry_msgs::PoseWithCovarianceStamped landmarkPose; 
        landmarkPose.header.stamp = ros::Time::now();
        landmarkPose.header.frame_id = "camera_landmark";
        landmarkPose.pose.pose.position.x = cameraToLandmarkTransform.translation().x()/1000;
        landmarkPose.pose.pose.position.y = cameraToLandmarkTransform.translation().y()/1000;
        landmarkPose.pose.pose.position.z = cameraToLandmarkTransform.translation().z()/1000;
         
        Eigen::Quaterniond cameraToLandmarkOrientation(cameraToLandmarkTransform.rotation());

        landmarkPose.pose.pose.orientation.x = cameraToLandmarkOrientation.x();
        landmarkPose.pose.pose.orientation.y = cameraToLandmarkOrientation.y();
        landmarkPose.pose.pose.orientation.z = cameraToLandmarkOrientation.z();
        landmarkPose.pose.pose.orientation.w = cameraToLandmarkOrientation.w();

        tf::Quaternion q(landmarkPose.pose.pose.orientation.x,landmarkPose.pose.pose.orientation.y,landmarkPose.pose.pose.orientation.z,landmarkPose.pose.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
         
        //file << "landmarkInCamera: x: " << landmarkPose.pose.pose.position.x << ", y: " << landmarkPose.pose.pose.position.y
          //   <<", z: "<< landmarkPose.pose.pose.position.z <<", roll: "<< roll <<", pitch: "<< pitch << ", yaw: "<< yaw << std::endl;
         
        //ROS_INFO_STREAM("Robot To Landmark: "<<robotToLandmark.translation()<<","<<robotToLandmark.rotation());
        updateLandmarkPose.header = landmarkPose.header;
        updateLandmarkPose.isNew = true;
        updateLandmarkPose.pose = landmarkPose;
        //file<<"return success"<<std::endl;
        return true;
      }

      else
      {
        ROS_WARN("Nothing received from Optimetre System");
        return false;
      }
    }

    bool GetLandmarkPose (landmarks_detection::GetLandmarkPose::Request &req, landmarks_detection::GetLandmarkPose::Response &res)
    {
      if(detect_landmark.getDetectionStatus(req.imgStamp) && computeLandmarkPose())
      {
        
        //cout << "req received" << endl;
        res.landmarkPose = updateLandmarkPose;
        return true;
      }

      else
      {
        //cout << "req not received" << endl;
        return false;
      }

    }

    //need review because of frams and transformartions , maybe only provide the qbove service and when needed apply the transformation in the desired robot frame 
    //and then get the inverse
    bool GetRobotToLandmarkPose (landmarks_detection::GetRobotToLandmarkPose::Request &req, landmarks_detection::GetRobotToLandmarkPose::Response &res)
    {
        component.updateData(data);
         if(newLandmarkObservationReceived(data.strTimeStamp))
        {
            //Extract the rotation matrix from the received data
            Eigen::Matrix3d robotOrientation;
            robotOrientation(0,0) = 0.0;//data.adFrameAttitude[0][0];
            robotOrientation(0,1) = 0.0;//data.adFrameAttitude[0][1];
            robotOrientation(0,2) = 0.0;//data.adFrameAttitude[0][2];
            robotOrientation(1,0) = 0.0;//data.adFrameAttitude[1][0];
            robotOrientation(1,1) = 0.0;//data.adFrameAttitude[1][1];
            robotOrientation(1,2) = 0.0;//data.adFrameAttitude[1][2];

            Eigen::Vector3d ea = robotOrientation.eulerAngles(0, 1, 2); 
            //ROS_INFO_STREAM("Euler angles: "<< ea);
            //TODO: add file for landmarks logs and check the timestamps and verify the angles by getting the inverse again
            /*
            Matrix3f n;
            n = AngleAxisf(ea[0], Vector3f::UnitX())
           *AngleAxisf(ea[1], Vector3f::UnitY())
           *AngleAxisf(ea[2], Vector3f::UnitZ()); 
            */
           //Extract the translation vector from the received data
          // Eigen::Translation<double,3> robotTranslation (data.adCameraCoordinates[0] ,data.adCameraCoordinates[1] ,data.adCameraCoordinates[2]);
           Eigen::Translation<double,3> robotTranslation (572,786,-1606);

           //Construct the affine transformation matrix including both rotation and translation [R T] --> to check here if i can use straight the rotation matrix
           Eigen::Quaterniond robotRotation (robotOrientation);
           Eigen::Transform <double,3,Eigen::Affine> robotToLandmark = robotTranslation*robotRotation;
           
        }
    }

    bool newLandmarkObservationReceived (std::string strTimeStamp)
    {   
       // ROS_INFO_STREAM("the time stamp is: "<<strTimeStamp);
        lastUpdateTime = strTimeStamp;
        if(lastUpdateTime=="00:00:00")
        {
            //ROS_ERROR("No new landmark updates received so far");
            return false;
        }
        else
        {
            //comapre strings given the time format , to check it later
            return true;
        }

    }

};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "landmark_detection_node");
  
  LandmarkDetection landmarkDetectionMgr;
   
  ros::NodeHandle nh;
  
  ros::ServiceClient getLandmarkPoseClnt_;
  landmarks_detection::GetLandmarkPose getlandmarkPoseSrv_;
  ros::Publisher landmarkPosePub_;
  ros::Subscriber kinectLandmarkDetectionSub, kinectRGBSub, kinectDepthSub;
    
  getLandmarkPoseClnt_ = nh.serviceClient<landmarks_detection::GetLandmarkPose>("/getLandmarkPose");      // service

  //landmarkPosePub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/landmark/pose",10);
  
  kinectLandmarkDetectionSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 50, boost::bind(&landmarkDetection::imageReceivedCallback, &detect_landmark, _1));
  
  //kinectRGBSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 50, boost::bind(&featureMatching::rgbReceivedCallback, &match_features, _1));
  //kinectDepthSub = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 50, boost::bind(&featureMatching::depthReceivedCallback, &match_features, _1));
  
  while(ros::ok())
  { 

    if(landmarkDetectionMgr.computeLandmarkPose())
    { 
        //this landmark pose is in camera_landmark frame, 
        //it needs to be converted to world or map frame in order to be processed in the EKF correctly
        landmarks_detection::LandmarkPose landmarkPose = landmarkDetectionMgr.getLandmarkUpdate();
        //TODO: has to be set from within the service when provided and later the message returned from the service is the one to be published
        //only the transformation applied, and in case it is not new dont publish 
        landmarkPosePub_.publish(landmarkPose.pose);
    }
      
      ros::spinOnce();
      sleep(1.0);
  }

 

    // Terminate the component
    if(landmarkDetectionMgr.getComponent().terminate())
    {
        std::cout << "The component is terminated." << std::endl;
    }

	return 0;
}
