/*!
 *  Dina Youakim
 */



#include "EncoderOdom.h"

EncoderOdom::EncoderOdom()
{

  ROS_INFO_STREAM("in encoder constructor");
   ros::NodeHandle private_nh("~");
   prev_x_front = 0.0; 
   prev_y_front = 0.0;
   prev_yaw_front = 0.0;
    prev_x_rear = 0.0;
    prev_y_rear = 0.0;
    prev_yaw_rear = 0.0;

    data.open("../encoder_rear_data.txt");
    data << "file initialised in EncoderOdom" << std::endl;

    linear_rear.open("../encoder_rear_linear.txt");
    linear_rear << "file initialised in EncoderOdom" << std::endl;

    angular_rear.open("../encoder_rear_angular.txt");
    angular_rear << "file initialised in EncoderOdom" << std::endl;

    linear_front.open("../encoder_front_linear.txt");
    linear_front << "file initialised in EncoderOdom" << std::endl;

    angular_front.open("../encoder_front_angular.txt");
    angular_front << "file initialised in EncoderOdom" << std::endl;

    odom.open("../robot_odom.txt");
    odom << "file initialised in EncoderOdom" << std::endl;

    //data<<"deltaL, deltaR,dist,time"<<std::endl;
    //linear<<"poseX,poseY,linear,linarVx, linearVy"<<std::endl;
    //angular<<"Yaw,angular, angular rate"<<std::endl;
    //odom<<"Pos0,Pos1,Pos2,Pos3, Diff0, Diff1, Diff2, Diff3"<<std::endl;
    firstTime = true;
    enc_left = std::numeric_limits<int>::max();
    enc_right = std::numeric_limits<int>::max();
    counter =0;
}

EncoderOdom::EncoderOdom(double wheelDis,double wheelRadius, double encoderOneCircleCnt)
{
  EncoderOdom();
  wheelDis_ = wheelDis;
  wheelRadius_ = wheelRadius;
  encoderOneCircleCnt_ = encoderOneCircleCnt;
  
}

EncoderOdom::~EncoderOdom()
{
}

void EncoderOdom::setParams(double dist,double rad,double count)
{
  wheelRadius_=rad;
  wheelDis_ = dist;
  encoderOneCircleCnt_ = count;
}

int EncoderOdom::start()
{

  frontEncoder_pub_ = node_.advertise<nav_msgs::Odometry>("front_motor_odom", 1);
  rearEncoder_pub_ = node_.advertise<nav_msgs::Odometry>("rear_motor_odom", 50);
  motor_sub_ = node_.subscribe<jaguar4x4_2014::MotorDataArray>("/drrobot_motor", 50, boost::bind(&EncoderOdom::motorEncoderReceived, this, _1));
  vis_pub = node_.advertise<visualization_msgs::Marker>("encoder_odom_marker", 1);
 
}

int EncoderOdom::stop()
{
  motor_sub_.shutdown();
  vis_pub.shutdown();
}

void EncoderOdom::motorEncoderReceived(const jaguar4x4_2014::MotorDataArray::ConstPtr& encoder_data)
{

  odom<<encoder_data->motorData[0].encoderPos<<","<<encoder_data->motorData[1].encoderPos<<","<<encoder_data->motorData[2].encoderPos<<","<<encoder_data->motorData[3].encoderPos<<","<<encoder_data->motorData[0].encoderDiff<<","<<encoder_data->motorData[1].encoderDiff<<","<<encoder_data->motorData[2].encoderDiff<<","<<encoder_data->motorData[3].encoderDiff<<std::endl;

  ros::Time current_time_encoder = ros::Time::now();
     /*
if(encoder_data->motorData[2].encoderPos<ENCODER_LOW_WRAP && prevRearLeftEncoder>ENCODER_HIGH_WRAP)
{
	lmult ++;
}

if(encoder_data->motorData[2].encoderPos>ENCODER_HIGH_WRAP && prevRearLeftEncoder<ENCODER_LOW_WRAP)
{
	lmult --;
}
int left = encoder_data->motorData[2].encoderPos + lmult*(ENCODER_MAX-ENCODER_MIN);

if(encoder_data->motorData[3].encoderPos<ENCODER_LOW_WRAP && prevRearRightEncoder>ENCODER_HIGH_WRAP)
{
	rmult ++;
}

if(encoder_data->motorData[3].encoderPos>ENCODER_HIGH_WRAP && prevRearRightEncoder<ENCODER_LOW_WRAP)
{
	rmult --;
}
int right = encoder_data->motorData[3].encoderPos + rmult*(ENCODER_MAX-ENCODER_MIN);

//if(encoder_data->motorData[0].encoderDiff!=0 && encoder_data->motorData[1].encoderDiff!=0)
//{
 if(firstTime)
  {
     //has to be fixed in the first reception of the encoder message
    last_time_encoder = ros::Time::now();
    prevRearLeftEncoder = encoder_data->motorData[2].encoderPos;
    prevRearRightEncoder = encoder_data->motorData[3].encoderPos;
    prevFrontLeftEncoder = encoder_data->motorData[0].encoderPos;
    prevFrontRightEncoder = encoder_data->motorData[1].encoderPos;
    firstTime = false;
  }
    //Compute front odom
     double delta_front_left = ((encoder_data->motorData[0].encoderPos-prevFrontLeftEncoder)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
     double delta_front_right = (((-encoder_data->motorData[1].encoderPos)-(-prevFrontRightEncoder))*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
    double front_dist = (delta_front_left + delta_front_right)/2;
    double delta_yaw_front = (delta_front_right - delta_front_left)/ wheelDis_;

    double delta_v_front = front_dist / (current_time_encoder-last_time_encoder).toSec() ;
    double delta_w_front =  delta_yaw_front/ (current_time_encoder-last_time_encoder).toSec() ;
    if(front_dist!=0)
    {
      double delta_x_front = cos(delta_yaw_front)*front_dist;
      double delta_y_front = -sin(delta_yaw_front)*front_dist;
      prev_x_front = prev_x_front + cos(delta_yaw_front)*delta_x_front-sin(delta_yaw_front)*delta_y_front;
      prev_y_front = prev_y_front + sin(delta_yaw_front)*delta_x_front+cos(delta_yaw_front)*delta_y_front;
    }

    if(delta_yaw_front!=0)
    {
      prev_yaw_front += delta_yaw_front;
    }

    //Crreate front odom message and publish
    geometry_msgs::Quaternion front_odom_quat = tf::createQuaternionMsgFromYaw(prev_yaw_front);
    nav_msgs::Odometry front_odom;
    front_odom.header.stamp = current_time_encoder;
    front_odom.header.frame_id = "front_odom";
    front_odom.pose.pose.position.x = prev_x_front;
    front_odom.pose.pose.position.y = prev_y_front;
    front_odom.pose.pose.position.z = 0;
    front_odom.pose.pose.orientation = front_odom_quat;

    front_odom.child_frame_id = "base_link";
    front_odom.twist.twist.linear.x = delta_v_front*cos(prev_yaw_front);
    front_odom.twist.twist.linear.y = delta_v_front*sin(prev_yaw_front);
    front_odom.twist.twist.angular.z = delta_w_front;

    frontEncoder_pub_.publish(front_odom);
*/
   
//}
//==========================================================================================//
  double delta_front_left = 0.0;
  double delta_front_right = 0.0;

  delta_front_left = ((encoder_data->motorData[0].encoderPos-(prevFrontLeftEncoder))*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
  delta_front_right = (((-encoder_data->motorData[1].encoderPos)-(-prevFrontRightEncoder))*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;

  double front_dist = (delta_front_left + delta_front_right)/2;
  double delta_yaw_front = (delta_front_right - delta_front_left)/ wheelDis_;

  double delta_x_front=0.0;
  double delta_y_front=0.0;
  double delta_v_front = front_dist / (current_time_encoder-last_time_encoder).toSec() ;
  double delta_w_front =  delta_yaw_front/ (current_time_encoder-last_time_encoder).toSec() ;
    

  if(front_dist!=0)
  {
    ROS_INFO_STREAM("here in front dist");
    delta_x_front = cos(delta_yaw_front)*front_dist;
    delta_y_front = -sin(delta_yaw_front)*front_dist;
    prev_x_front = prev_x_front + (cos(prev_yaw_front)*delta_x_front-sin(prev_yaw_front)*delta_y_front);
    prev_y_front = prev_y_front + (sin(prev_yaw_front)*delta_x_front+cos(prev_yaw_front)*delta_y_front);
  }
    
    prev_yaw_front += delta_yaw_front;
  
    if(delta_yaw_front!=0)
    {
      ROS_INFO_STREAM("here in front yaw");
      
      //increase the correction factor when increasing the sleep time (getting less frequent messaages thus needs higher correction) from tests: for 0.5 sleep a correction of 0.05 is good
      //for 1.0 sleep a corretion of 0.15 works, it is also subject to the ground used for testing
      if(encoder_data->motorData[0].encoderPos>prevFrontLeftEncoder && encoder_data->motorData[1].encoderPos>prevFrontRightEncoder)
      {
        prev_yaw_front +=0.0;//0.05;
        //ROS_INFO_STREAM("EncoderOdom: turn righttttt");
        //data<<"turn righttttt"<<std::endl;
      }
      else
      {
        prev_yaw_front -=0.0; 
        //ROS_INFO_STREAM("EncoderOdom: turn lefttttt");
        //data<<"turn lefttttt"<<std::endl;
      }
    }
 if (prev_yaw_front>=M_PI)
    prev_yaw_front = prev_yaw_front - 2*M_PI;
  else if (prev_yaw_front<=-M_PI)
    prev_yaw_front = prev_yaw_front + 2*M_PI;

//==========================================================================================//

  //Compute rear odom
  double delta_rear_left = 0.0;
  double delta_rear_right = 0.0;
  data<<"new encoder data received at: "<<ros::Time::now()<<" ,acquired at: "<<encoder_data->motorData[2].header.stamp<<std::endl;
  data<<encoder_data->motorData[0].encoderPos<<","<<encoder_data->motorData[1].encoderPos<<","<<encoder_data->motorData[2].encoderPos<<","<<encoder_data->motorData[3].encoderPos<<std::endl;
 
  
  /*if ((encoder_data->motorData[2].encoderPos>prevRearLeftEncoder && encoder_data->motorData[3].encoderPos>prevRearRightEncoder) || (encoder_data->motorData[2].encoderPos<prevRearLeftEncoder && encoder_data->motorData[3].encoderPos<prevRearRightEncoder))
  {
    delta_rear_left = ((encoder_data->motorData[2].encoderPos-prevRearLeftEncoder)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
    delta_rear_right = ((encoder_data->motorData[3].encoderPos-prevRearRightEncoder)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
  }
  else{*/
     delta_rear_left = ((encoder_data->motorData[2].encoderPos-prevRearLeftEncoder)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
     delta_rear_right = (((-encoder_data->motorData[3].encoderPos)-(-prevRearRightEncoder))*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
  //}
  data<<"delta encoders:"<<delta_rear_left<<","<<delta_rear_right<<std::endl;
  data<<"previous values: "<<prev_x_rear<<","<<prev_y_rear<<","<<prev_yaw_rear<<std::endl;
  /*delta_rear_left = ((left-enc_left)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
    delta_rear_right = ((right-enc_right)*2*M_PI*wheelRadius_)/encoderOneCircleCnt_;
    enc_left = left ;
    enc_right = right;*/
    double rear_dist = (delta_rear_left + delta_rear_right)/2;
    double delta_yaw_rear = (delta_rear_right - delta_rear_left)/ wheelDis_;
    /*double yaw_rear =  prev_yaw_rear + delta_yaw_rear;
    double x_rear = prev_x_rear + rear_dist*cos(yaw_rear);
    double y_rear= prev_y_rear + rear_dist*sin(yaw_rear);
    double ang_rate = yaw_rear/(current_time_encoder-last_time_encoder).toSec() ;
    double vel_rear_left = (delta_rear_left )/(current_time_encoder-last_time_encoder).toSec() ;
    double vel_rear_right = (delta_rear_right )/(current_time_encoder-last_time_encoder).toSec() ;
    //double vel_linear_rear = (vel_rear_right + vel_rear_left)/2;
    //double vel_angular_rear =  (vel_rear_right - vel_rear_left)/2;

    double vel_linear_rear,vel_angular_rear ;
    if (vel_rear_left==vel_rear_right)
          vel_angular_rear = 0;
    else
    {
      double r = (vel_rear_left*wheelDis_)/ (vel_rear_right - vel_rear_left);
      vel_angular_rear =  vel_rear_left/ r;
      vel_linear_rear = vel_angular_rear*(r+(wheelDis_/2));
    }*/
    double delta_x=0.0;
    double delta_y=0.0;
    double delta_v = rear_dist / (current_time_encoder-last_time_encoder).toSec() ;
    double delta_w =  delta_yaw_rear/ (current_time_encoder-last_time_encoder).toSec() ;
    

    if(rear_dist!=0)
    {
      ROS_INFO_STREAM("here in rear dist");
      delta_x = cos(delta_yaw_rear)*rear_dist;
      delta_y = -sin(delta_yaw_rear)*rear_dist;
      prev_x_rear = prev_x_rear + (cos(prev_yaw_rear)*delta_x-sin(prev_yaw_rear)*delta_y);
      prev_y_rear = prev_y_rear + (sin(prev_yaw_rear)*delta_x+cos(prev_yaw_rear)*delta_y);
    }
    
    prev_yaw_rear += delta_yaw_rear;
  
    if(delta_yaw_rear!=0)
    {
      ROS_INFO_STREAM("here in rear yaw");
      
      //increase the correction factor when increasing the sleep time (getting less frequent messaages thus needs higher correction) from tests: for 0.5 sleep a correction of 0.05 is good
      //for 1.0 sleep a corretion of 0.15 works, it is also subject to the ground used for testing
      if(encoder_data->motorData[2].encoderPos>prevRearLeftEncoder && encoder_data->motorData[3].encoderPos>prevRearRightEncoder)
      {
        prev_yaw_rear +=0.1;//0.05;
        ROS_INFO_STREAM("turn righttttt");
        data<<"turn righttttt"<<std::endl;
      }
      else
      {
        prev_yaw_rear -=0.1; 
        ROS_INFO_STREAM("turn lefttttt");
        data<<"turn lefttttt"<<std::endl;
      }
    }
  if (prev_yaw_rear>M_PI)
    prev_yaw_rear = prev_yaw_rear - 2*M_PI;
  else if (prev_yaw_rear<-M_PI)
    prev_yaw_rear = prev_yaw_rear + 2*M_PI;
  //Crreate front odom message and publish
  geometry_msgs::Quaternion rear_odom_quat = tf::createQuaternionMsgFromYaw(prev_yaw_front);//prev_yaw_rear+prev_yaw_front)/2
  nav_msgs::Odometry rear_odom;
  rear_odom.header.stamp = current_time_encoder;
  rear_odom.header.frame_id = "odom";
  rear_odom.pose.pose.position.x = prev_x_front;//(prev_x_rear+prev_x_front)/2;
  rear_odom.pose.pose.position.y = prev_y_front;//(prev_y_rear+prev_y_front)/2;
  rear_odom.pose.pose.position.z = 0;
  rear_odom.pose.pose.orientation = rear_odom_quat;

  rear_odom.child_frame_id = "base_link";
  rear_odom.twist.twist.linear.x = delta_v*cos(prev_yaw_rear);
  rear_odom.twist.twist.linear.y = delta_v*sin(prev_yaw_rear);
  rear_odom.twist.twist.angular.z = delta_w;

  rearEncoder_pub_.publish(rear_odom);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "encoder";
  marker.id = counter;
  counter++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = rear_odom.pose.pose.position.x;
  marker.pose.position.y = rear_odom.pose.pose.position.y;
  marker.pose.position.z = rear_odom.pose.pose.position.z;
  marker.pose.orientation.x = rear_odom.pose.pose.orientation.x;
  marker.pose.orientation.y = rear_odom.pose.pose.orientation.y;
  marker.pose.orientation.z = rear_odom.pose.pose.orientation.z;
  marker.pose.orientation.w = rear_odom.pose.pose.orientation.w;
  marker.scale.x = 0.2;
  marker.scale.y = 0.008;
  marker.scale.z = 0.008;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  vis_pub.publish( marker );

  // }


  //data<<"REAR:"<<delta_rear_left<<","<<delta_rear_right<<","<<rear_dist<<","<<(current_time_encoder-last_time_encoder).toSec()<<std::endl;
  //data<<"FRONT:"<<delta_front_left<<","<<delta_front_right<<","<<front_dist<<","<<(current_time_encoder-last_time_encoder).toSec()<<std::endl;


  linear_rear<<prev_x_rear<<","<<prev_y_rear<<","<<delta_v*cos(prev_yaw_rear)<<","<<delta_v*sin(prev_yaw_rear)<<","<<rear_odom.header.stamp<<","<<front_dist<<std::endl;
  angular_rear<<prev_yaw_rear<<","<<delta_yaw_rear<<","<<rear_odom.header.stamp<<std::endl;//","<<delta_yaw_rear<<std::endl;
  
  linear_front<<prev_x_front<<","<<prev_y_front<<","<<delta_v_front*cos(prev_yaw_front)<<","<<delta_v_front*sin(prev_yaw_front)<<","<<rear_odom.header.stamp<<","<<rear_dist<<std::endl;
  angular_front<<prev_yaw_front<<","<<delta_yaw_front<<","<<rear_odom.header.stamp<<std::endl;

  last_time_encoder = current_time_encoder;
  prevRearLeftEncoder = encoder_data->motorData[2].encoderPos;
  prevRearRightEncoder = encoder_data->motorData[3].encoderPos;
  prevFrontLeftEncoder = encoder_data->motorData[0].encoderPos;
  prevFrontRightEncoder = encoder_data->motorData[1].encoderPos;
  //sleep(0.5);

}

  
