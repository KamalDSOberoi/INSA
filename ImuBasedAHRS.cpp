/*!
 *  Dina Youakim
 */



#include "ImuBasedAHRS.h"

ImuBasedAHRS::ImuBasedAHRS()
{
   ros::NodeHandle private_nh("~");
   currentCalibratedIMU = new float[12];
   // initialize quaternion
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
    exInt = 0.0;
    eyInt = 0.0;
    ezInt = 0.0;
    twoKp = twoKpDef;
    twoKi = twoKiDef;
    integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
    lastUpdate = 0;
    now = 0;
    file.open("../ahrs.txt");
    file<<"file initialised in ImuBasedAHRS" << std::endl;

    file2.open("../ahrs_data.txt");
    file2<<"file initialised in ImuBasedAHRS" << std::endl;
    initial=true;
}

ImuBasedAHRS::ImuBasedAHRS(double wheelDis,double wheelRadius, double encoderOneCircleCnt)
{
    ImuBasedAHRS();
    wheelDis_ = wheelDis;
    wheelRadius_ = wheelRadius;
    encoderOneCircleCnt_ = encoderOneCircleCnt;
}

ImuBasedAHRS::~ImuBasedAHRS()
{
}

void ImuBasedAHRS::setParams(double gyro_x,double gyro_y,double gyro_z)
{
  gyroX_offset = gyro_x;
  gyroY_offset = gyro_y;
  gyroZ_offset = gyro_z;
}

void ImuBasedAHRS::setWheelRadius(double rad)
{
   wheelRadius_=rad;
}

void ImuBasedAHRS::setWheelDist(double dist)
{
   wheelDis_ = dist;
}

void ImuBasedAHRS::setOneCircleCnt(double count)
{
   encoderOneCircleCnt_ = count;
}


int ImuBasedAHRS::start()
{
  imu_pub_= node_.advertise<sensor_msgs::Imu>("imu", 50);
  jaguar_imu_sub_ = node_.subscribe<imu_encoders_calibration::IMUCalibrated>("jaguar_calibrated_imu", 1, boost::bind(&ImuBasedAHRS::imuCalibratedReceived, this, _1));
  yaw_pub_= node_.advertise<nav_msgs::Odometry>("yaw", 1);
  //has to be fixed in the first reception of the encoder message
  last_time_encoder = ros::Time::now();
  ROS_INFO_STREAM("START DONE");
}

int ImuBasedAHRS::stop()
{
  jaguar_imu_sub_.shutdown();
}

void ImuBasedAHRS::imuCalibratedReceived(const imu_encoders_calibration::IMUCalibrated::ConstPtr& imu_calibrated)
{
    //ROS_INFO_STREAM("IMUBasedAHRS: IMU received");
    frame = imu_calibrated->header.frame_id;
    currentCalibratedIMU[0] = imu_calibrated->accel_x;
    currentCalibratedIMU[1] = imu_calibrated->accel_y;
    currentCalibratedIMU[2] = imu_calibrated->accel_z;
    //ROS_INFO_STREAM("IMUBasedAHRS: IMU acceleration received");
    currentCalibratedIMU[3] = imu_calibrated->gyro_x;
    currentCalibratedIMU[4] = imu_calibrated->gyro_y;
    currentCalibratedIMU[5] = imu_calibrated->gyro_z;
    //ROS_INFO_STREAM("IMUBasedAHRS: IMU gyro received");
    currentCalibratedIMU[6] = imu_calibrated->comp_x;
    currentCalibratedIMU[7] = imu_calibrated->comp_y;
    currentCalibratedIMU[8] = imu_calibrated->comp_z;
    currentCalibratedIMU[9] = imu_calibrated->yaw;
    currentCalibratedIMU[10] = imu_calibrated->pitch;
    currentCalibratedIMU[11] = imu_calibrated->roll;

    //ROS_INFO_STREAM("IMUBasedAHRS: before creating new IMU");
/*ROS_INFO_STREAM(imu_calibrated->accel_x<<","<<imu_calibrated->accel_y<<","<<imu_calibrated->accel_z<<";"
<<imu_calibrated->gyro_x<<","<<imu_calibrated->gyro_y<<","<<imu_calibrated->gyro_z);*/
    createImuMessage();
}

// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from Sebastian Madgwick filter which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//=====================================================================================================
void ImuBasedAHRS::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;

  // Auxiliary variables to avoid repeated arithmetic
  q0q0 = q0 * q0;
  q0q1 = q0 * q1;
  q0q2 = q0 * q2;
  q0q3 = q0 * q3;
  q1q1 = q1 * q1;
  q1q2 = q1 * q2;
  q1q3 = q1 * q3;
  q2q2 = q2 * q2;
  q2q3 = q2 * q3;
  q3q3 = q3 * q3;
  
  
  
  // Use magnetometer measurement only when valid (avoids NaN in magnetometer normalisation)
  if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
    float hx, hy, bx, bz;
    float halfwx, halfwy, halfwz;
    
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    
    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    
    // Estimated direction of magnetic field
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
    
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex = (my * halfwz - mz * halfwy);
    halfey = (mz * halfwx - mx * halfwz);
    halfez = (mx * halfwy - my * halfwx);
  }
  

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f)) {
    float halfvx, halfvy, halfvz;
    
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
  
    // Error is sum of cross product between estimated direction and measured direction of field vectors
    halfex += (ay * halfvz - az * halfvy);//(ay*0.5)
    halfey += (az * halfvx - ax * halfvz);//(-ax*0.5)
    halfez += (ax * halfvy - ay * halfvx);//(0)
  }
  //ROS_INFO_STREAM("hlaf values: "<< halfex<<","<<halfey<<","<< halfez);
  // Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
//ROS_INFO_STREAM("all non zero values");
    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f) {
      integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
      integralFBy += twoKi * halfey * (1.0f / sampleFreq);
      integralFBz += twoKi * halfez * (1.0f / sampleFreq);
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }
  
  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
  gy *= (0.5f * (1.0f / sampleFreq));
  gz *= (0.5f * (1.0f / sampleFreq));
//ROS_INFO_STREAM("gyrooz"<<gx<<","<<gy<<","<<gz);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);
  //ROS_INFO_STREAM("Qs before normalization"<<q0<<","<<q1<<","<<q2<<","<<q3);
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
//ROS_INFO_STREAM("Qs after normalization"<<q0<<","<<q1<<","<<q2<<","<<q3);
file2<<sampleFreq<<","<<recipNorm<<","<<qa<<","<<qb<<","<<qc<<","<<q0<<","<<q1<<","<<q2<<","<<q3<<gx<<","<<gy<<","<<gz<<std::endl;
}


void ImuBasedAHRS::getQ(float * q) {
  
  /*
  DEBUG_PRINT(val[3] * M_PI/180);
  DEBUG_PRINT(val[4] * M_PI/180);
  DEBUG_PRINT(val[5] * M_PI/180);
  DEBUG_PRINT(val[0]);
  DEBUG_PRINT(val[1]);
  DEBUG_PRINT(val[2]);
  DEBUG_PRINT(val[6]);
  DEBUG_PRINT(val[7]);
  DEBUG_PRINT(val[8]);
  */
  
  
  now = ros::Time::now().toSec();
  sampleFreq = 1.0 / ((now - lastUpdate));
  lastUpdate = now;
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  //AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
  // use the call below when using a 6DOF IMU
  AHRSupdate(currentCalibratedIMU[3] * M_PI/180, currentCalibratedIMU[4] * M_PI/180, currentCalibratedIMU[5] * M_PI/180, currentCalibratedIMU[0], currentCalibratedIMU[1], currentCalibratedIMU[2], currentCalibratedIMU[6], currentCalibratedIMU[7], currentCalibratedIMU[8]);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void ImuBasedAHRS::getEuler(float * angles) {
float q[4]; // quaternion
  getQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}


void ImuBasedAHRS::getAngles(float * angles) {
  float a[3]; //Euler
  getEuler(a);

  angles[0] = a[0];
  angles[1] = a[1];
  angles[2] = a[2];
  
  if(angles[0] < 0)angles[0] += 360;
  if(angles[1] < 0)angles[1] += 360;
  if(angles[2] < 0)angles[2] += 360;

}


void ImuBasedAHRS::getYawPitchRoll(float * ypr,float*q) {
 // float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  //getQ(q);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz));
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz));
  float initial_correction=0.0;
  /*if (initial && abs(currentCalibratedIMU[5]*180/M_PI)<1)
 {
	initial_correction = ypr[0];
        initial = false;
 }
if(abs(currentCalibratedIMU[5]*180/M_PI)<1)
	ypr[0] = 0;
else
	ypr[0]-=initial_correction;*/
 if (ypr[0]>M_PI)
		ypr[0] = ypr[0] - 2*M_PI;
          else if (ypr[0]<-M_PI)
		ypr[0] = ypr[0] + 2*M_PI;
}


float ImuBasedAHRS::invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

void ImuBasedAHRS::createImuMessage() 
{
    float* quaternion = new float[4];
    getQ(quaternion);
    tf::Quaternion q(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);	
    float * ypr= new float[3];
    getYawPitchRoll(ypr,quaternion);
    file<<tf::getYaw(q)<<","<<ypr[0]<<","<< currentCalibratedIMU[3]<<","<< currentCalibratedIMU[4]<<","<< currentCalibratedIMU[5]*180/M_PI<<","<<ros::Time::now()<<std::endl;
    
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = frame;
    /*imu.orientation.x = quaternion[0];
    imu.orientation.y = quaternion[1];
    imu.orientation.z = quaternion[2];
    imu.orientation.w = quaternion[3];*/
    imu.orientation = tf::createQuaternionMsgFromYaw(currentCalibratedIMU[9]);
    imu.angular_velocity.x = currentCalibratedIMU[3];
    imu.angular_velocity.y = currentCalibratedIMU[4];
    imu.angular_velocity.z = currentCalibratedIMU[5];

    imu.linear_acceleration.x = currentCalibratedIMU[0];
    imu.linear_acceleration.y = currentCalibratedIMU[1];
    imu.linear_acceleration.z = currentCalibratedIMU[2];

    imu_pub_.publish(imu);

    nav_msgs::Odometry yaw_odom;
    yaw_odom.header.stamp =  imu.header.stamp;
    yaw_odom.header.frame_id = frame;
    yaw_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(currentCalibratedIMU[9]);
    yaw_pub_.publish(yaw_odom);
    
}
