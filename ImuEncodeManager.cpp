#include  "EncoderOdom.h"
#include  "ImuBasedAHRS.h"
#include  "IMUCalibration.h"

double gyroX_offset, gyroY_offset, gyroZ_offset;
int  encoderOneCircleCnt_;
double wheelDis_, wheelRadius_;
ros::Subscriber statusSub_;
EncoderOdom* encodersOdom;
IMUCalibration* imu_calibrate;
ImuBasedAHRS* AHRS ;

void robotStatusCallback(const std_msgs::String::ConstPtr& robotStatus)
{
    if(robotStatus->data=="moving")
    {
       encodersOdom->start();
       imu_calibrate->start();
       AHRS->start();
       ROS_INFO_STREAM("MOTION COMMAND!!!");
    }
    if(robotStatus->data=="stopped")
    {
       encodersOdom->stop();
       imu_calibrate->stop();
       AHRS->stop();
    }
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "jaguar_imu_encoders_node");
	ros::NodeHandle nh;
  sleep(21);
  
  std::ofstream file;
  file.open("../wheel_odometry.txt");
  file << "file initialised in IMUEncodeManager" << std::endl;
  

  wheelDis_ = 0.5;
	nh.getParam("WheelDistance",wheelDis_);

  encoderOneCircleCnt_ = 285;
  nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);

  wheelRadius_ = 0.127;
  nh.getParam("WheelRadius", wheelRadius_);

  nh.getParam("GyroX_offset", gyroX_offset);
  
  nh.getParam("GyroY_offset", gyroY_offset);
  
  nh.getParam("GyroZ_offset", gyroZ_offset);
  ROS_INFO_STREAM("before params set");
  

  file<<"Parameters: WheelDistance:"<<wheelDis_<<", EncoderCircleCnt:"<<encoderOneCircleCnt_<<", WheelRadius:"<<wheelRadius_<<", GyroX_offset:"<<gyroX_offset
      <<", GyroY_offset:"<<gyroY_offset<<", GyroZ_offset:" << gyroZ_offset << std::endl;
  
  statusSub_ = nh.subscribe<std_msgs::String>("/drrobot_status", 1, &robotStatusCallback);
  
  encodersOdom = new EncoderOdom();
  imu_calibrate = new IMUCalibration();
  AHRS = new ImuBasedAHRS();
  
  encodersOdom->setParams(wheelDis_, wheelRadius_, encoderOneCircleCnt_);
  imu_calibrate->initGyro(gyroX_offset, gyroY_offset, gyroZ_offset);
  AHRS->setParams(gyroX_offset, gyroY_offset, gyroZ_offset);
  
  ros::spin();
  
  return(0);

}
