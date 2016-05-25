#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <fstream>

#define MOTOR_NUM           4       //max

ros::Publisher pub_,robotStatusPub_;
std::ofstream ekf;

void moveForward()
{
  std_msgs::String cmdVel_;
  
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  std::stringstream ss;
  ss << "MMW !M 140 -140";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  sleep(1);
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void moveBackward()
{
  std_msgs::String cmdVel_;
  
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  //pub_.publish(cmdVel_);
  //sleep(1);
  std::stringstream ss;
  ss << "MMW !M -140 140";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  sleep(1);
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void turnLeft ()
{
std_msgs::String cmdVel_;
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;

  ss << "MMW !M -350 -350";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(1);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void rotateLeft ()
{
std_msgs::String cmdVel_;
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;
  ss << "MMW !M -200 -200";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(3);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void turnRight ()
{
  std_msgs::String cmdVel_;
  ros::Duration d(1,5);
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;

  ss << "MMW !M 305 305";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(1.0);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void rotateRight ()
{
  std_msgs::String cmdVel_;
  ros::Duration d(1,5);
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;

  ss << "MMW !M 205 205";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(1);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void forwardRotateRight ()
{
  std_msgs::String cmdVel_;
  ros::Duration d(1,5);
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;
  ss << "MMW !M 150 250";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(1.0);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void forwardRotateLeft ()
{
  std_msgs::String cmdVel_;
  int forwardCnt = 0; 
  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss;
  ss << "MMW !M -150 -250";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  ROS_INFO_STREAM("start: "<<ros::Time::now());
  sleep(1.0);
  ROS_INFO_STREAM("end: "<<ros::Time::now());
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}

void corridorMotion()
{
  std_msgs::String cmdVel_;
  int forwardCnt = 0; 
  std::stringstream ss0,sStop,sCmd;
  ss0 << "MMW !MG";
  sStop << "MMW !M 0 0";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  //Rotate Left
  sCmd << "MMW !M -170 -220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //Forward
  sCmd.str("");
  sCmd << "MMW !M 200 -200";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //RotateRight
  sCmd.str("");
  sCmd << "MMW !M 170 220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1);
  //RotateRight
  sCmd.str("");
  sCmd << "MMW !M 170 220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1);
  //Forward
  sCmd.str("");
  sCmd << "MMW !M 200 -200";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //Rotate Left
  sCmd.str("");
  sCmd << "MMW !M -170 -220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //Forward
  sCmd.str("");
  sCmd << "MMW !M 200 -200";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //RotateRight
  sCmd.str("");
  sCmd << "MMW !M 170 220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1);
  //Rotate Left
  sCmd.str("");
  sCmd << "MMW !M -170 -220";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //Forward
  sCmd.str("");
  sCmd << "MMW !M 305 -305";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //Rotate Left
  sCmd.str("");
  sCmd << "MMW !M -200 -300";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1.0);
  //RotateRight
  sCmd.str("");
  sCmd << "MMW !M 200 300";
  cmdVel_.data = sCmd.str();
  pub_.publish(cmdVel_);
  sleep(1);
  //STOP
  cmdVel_.data = sStop.str();
  pub_.publish(cmdVel_);
  sleep(1);

}

void turn360()
{
  std_msgs::String cmdVel_;

  std::stringstream ss0;
  ss0 << "MMW !MG";
  cmdVel_.data = ss0.str();
  pub_.publish(cmdVel_);
  sleep(1);
  std::stringstream ss1;
  ss1 << "MMW !M 0 0";
  cmdVel_.data = ss1.str();
  std::stringstream ss;
  ss << "MMW !M 270 270";
  cmdVel_.data = ss.str();
  pub_.publish(cmdVel_);
  sleep(2);
  cmdVel_.data = ss1.str();
  pub_.publish(cmdVel_);
  sleep(1);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_vel_node");

    ros::NodeHandle n;
    pub_ = n.advertise<std_msgs::String>("drrobot_motor_cmd", 1);
    robotStatusPub_ = n.advertise<std_msgs::String>("drrobot_status", 1);
    sleep(5);
     //sleep(132);
    std_msgs::String cmdVel_;
    int forwardCnt = 0; 

    /////////////////////////////////////////////////////////////////

    //while(ros::ok())     
    //{
      /*std_msgs::String status;
      std::stringstream ss0,ss1,ss2;
      ss0<<"moving";
      ss1<<"stopped";
      ss2<<"plot";
      status.data = ss0.str();
      robotStatusPub_.publish(status);*/


      //Kamal D S Oberoi
      sleep(20);                  // sleep until gyroscope is calibrated

      moveForward();
      sleep(3);
      //moveForward();
      

      /*status.data = ss1.str();
      sleep(1);
      robotStatusPub_.publish(status);
      status.data = ss2.str();
      sleep(1);
      robotStatusPub_.publish(status);*/
      //moveBackward();
      ros::spinOnce();
      // loop_rate.sleep();
    //}

    
    return(0);
}

