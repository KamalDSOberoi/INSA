/*!
 *  Dina Youakim
 */



#include "IMUCalibration.h"

IMUCalibration::IMUCalibration()
{
	ros::NodeHandle private_nh("~");
	initAcc();
	last_time = 0;
	file.open("../calibrated_imu.txt");
	file << "file initialised in IMUCalibration" << std::endl;

	yaw.open("../raw_yaw.txt");
	yaw << "file initialised in IMUCalibration" << std::endl;
	
	compass.open("../compass.txt");
	compass << "file initialised in IMUCalibration" << std::endl;
}

IMUCalibration::IMUCalibration(double gyroX_offset, double gyroY_offset, double gyroZ_offset)
{
	IMUCalibration();
	initGyro(gyroX_offset, gyroY_offset, gyroZ_offset);
}

IMUCalibration::~IMUCalibration()
{

}


int IMUCalibration::start()
{
	// publisher message to be changed
	jaguarCalibratedIMU_pub_ = node_.advertise<imu_encoders_calibration::IMUCalibrated>("jaguar_calibrated_imu", 50);
	jaguarIMUData_sub_ = node_.subscribe<jaguar4x4_2014::IMUData>("/drrobot_imu", 50, boost::bind(&IMUCalibration::imuDataReceived, this, _1));
	last_time = ros::Time::now().toSec();
}

int IMUCalibration::stop()
{
	jaguarIMUData_sub_.shutdown();
}

void IMUCalibration::imuDataReceived(const jaguar4x4_2014::IMUData::ConstPtr& imu_data)
{
	//ROS_INFO_STREAM("IMU data received");
	double time_elapsed =  ros::Time::now().toSec()- last_time;
	
	//compass<<imu_data->accel_x<<", "<<imu_data->accel_y<<", "<<imu_data->accel_z<<std::endl;
	double* gyro = calibrateGyro (imu_data->gyro_x, imu_data->gyro_y, imu_data->gyro_z, time_elapsed);
	double* acc = calibrateAccelerometer(imu_data->accel_x, imu_data->accel_y, imu_data->accel_z);
	double* comp = calibrateCompass(imu_data->comp_x, imu_data->comp_y, imu_data->comp_z);

	imu_encoders_calibration::IMUCalibrated calibratedData;
	calibratedData.header.frame_id = "base_link";
    calibratedData.header.seq = imu_data->header.seq;
    calibratedData.header.stamp = ros::Time::now();
	//Leave RPY as is, raw data
	double yawWrapped = -imu_data->yaw;
	if(yawWrapped>=M_PI)
		calibratedData.yaw = yawWrapped - 2*M_PI;
	else if (yawWrapped<=-M_PI)
		calibratedData.yaw = yawWrapped + 2*M_PI;

	calibratedData.yaw = yawWrapped;
	calibratedData.pitch = imu_data->pitch;
	calibratedData.roll = imu_data->roll;
	yaw<<calibratedData.yaw<<","<<calibratedData.header.stamp<<std::endl;//<<","<<imu_data->pitch<<","<<imu_data->roll<<std::endl;
	
	// gyro calibrated
	calibratedData.gyro_x = gyro[0]*M_PI/180;
	calibratedData.gyro_y = gyro[1]*M_PI/180;
	calibratedData.gyro_z = gyro[2]*M_PI/180;
	//file<<calibratedData.gyro_z<<","<<calibratedData.header.stamp<<std::endl;//<<","<<gyro[1]<<","<<gyro[2]<<std::endl;
	
	//acc calibrated
	calibratedData.accel_x = acc[0];
	calibratedData.accel_y = acc[1];
	calibratedData.accel_z = acc[2]-2.5;

	//compass calibrated
	calibratedData.comp_x = comp[0];
	calibratedData.comp_y = comp[1];
	calibratedData.comp_z = comp[2];

	compass<<imu_data->comp_x<<","<<imu_data->comp_y<<","<<imu_data->comp_z<<std::endl;
	compass<<comp[0]<<","<<comp[1]<<","<<comp[2]<<std::endl;
	float heading = atan2(calibratedData.comp_y, calibratedData.comp_x);

    // wrapping
     if (heading>M_PI)
		heading = heading - 2*M_PI;
          else if (heading<-M_PI)
		heading = heading + 2*M_PI;
    //if(heading < 0) heading += 2*M_PI;
    //if(heading > 2*M_PI) heading -= 2*M_PI;
    //heading *= (180/M_PI);
    //calibratedData.accel_z = heading;
    //ROS_INFO_STREAM("HEADING: "<<heading);
    compass<<heading<<std::endl;
	//ROS_INFO_STREAM("Calibrated message: "<<calibratedData);
	jaguarCalibratedIMU_pub_.publish (calibratedData);
	last_time = ros::Time::now().toSec();
//sleep(0.5);
}

void IMUCalibration::initGyro(double gyroX_offset, double gyroY_offset, double gyroZ_offset)
{
	ROS_INFO_STREAM("IMUCalibration: gyroX_offset:"<<gyroX_offset<<", gyroY_offset:"<<gyroY_offset<<", gyroZ_offset:"<<gyroZ_offset);
	gyro_gains[0] = gyro_gains[1] = gyro_gains[2] = 1.0;
	gyro_offsets[0] = gyroX_offset;
	gyro_offsets[1] = gyroY_offset;
	gyro_offsets[2] = gyroZ_offset;
	file << "gyro_offsets: x:"<<gyro_offsets[0]<<", y:"<<gyro_offsets[1]<<", z:"<<gyro_offsets[2]<<std::endl;
	gyro_polarities[0] = gyro_polarities[1] = gyro_polarities[2] = 1.0;
}

void IMUCalibration::initAcc ()
{
	acc_gains[0] = 0.00390625;//0.00376390;
  	acc_gains[1] = 0.00390625;//0.00376009;
  	acc_gains[2] = 0.00390625;//0.00349265;
}

double* IMUCalibration::calibrateGyro(double gyro_x, double gyro_y, double gyro_z, double timeElapsed)
{
	double *xyz = new double[3];
	xyz[0] = (gyro_x+gyro_offsets[0])/14.375 * gyro_polarities[0] * gyro_gains[0];
	xyz[1] = (gyro_y+gyro_offsets[1])/14.375 * gyro_polarities[1] * gyro_gains[1];
	xyz[2] = (gyro_z+gyro_offsets[2])/14.375 * gyro_polarities[2] * gyro_gains[2];
	//ROS_INFO_STREAM("calibrated GYRO: "<<xyz[0]<<","<<xyz[1]<<","<<xyz[2]);
	return xyz;
}

double* IMUCalibration::calibrateAccelerometer(double acc_x, double acc_y, double acc_z)
{
	double *xyz = new double[3];
    xyz[0] = acc_x * acc_gains[0];
    xyz[1] = acc_y * acc_gains[1];
    xyz[2] = acc_z * acc_gains[2];
    return xyz;
}

double* IMUCalibration::calibrateCompass(double comp_x, double comp_y, double comp_z)
{
	double *xyz = new double[3];
    xyz[0] = comp_x * COMPASS_SCALE;
    xyz[1] = comp_y * COMPASS_SCALE;
    xyz[2] = comp_z * COMPASS_SCALE;
    return xyz;
}




