#include "controlsegway.h"
#include "controlsegwayCmd.h"
#include <fstream>
#include <iostream>
using namespace std;
#define MAX_SIZE 1000
char inputString[MAX_SIZE];

controlsegway::controlsegway(rDC rdc)
#ifdef _USE_RCONTROLALGORITHM_EX_
	: rControlAlgorithmEx(rdc)
#else
	: rControlAlgorithm(rdc)
#endif
{
}

controlsegway::~controlsegway()
{
}


void controlsegway::_arrangeJointDevices()
{
}

void controlsegway::init(int mode)
{
	_lmotor = findDevice(_T("lmotor"));
	_rmotor = findDevice(_T("rmotor"));
	_tmotor = findDevice(_T("tmotor"));
	_gyro = findDevice(_T("gyro"));
	_enc_tilt = findDevice(_T("enc_tilt"));
	_tacho_lwheel = findDevice(_T("tacho_lwheel"));
	_tacho_rwheel = findDevice(_T("tacho_rwheel"));

	_lWheel_des = 0.0;
	_rWheel_des = 0.0;

	_tmotor_des = 0.0;

	_virtualmotor_bar_des = 0.0;

	//센서 devices의 출력값 초기화
	_tilt_theta = 0.0;

	for (int i = 0; i<3; i++)
		_gyro_buf[i] = 0.0;
	_lwheel_vel_out = 0.0;
	_rwheel_vel_out = 0.0;

	//PD제어를 위한 변수 초기값 설정
	Kp_pitch = -100;
	Kd_pitch = -1.5;
	Ki_pitch = -0;


	dt = 0.005;

	Kp_vel = -80;//100;
	Kd_vel = -0.1;//0;
	Ki_vel = 0;//0;

	K_diff = 30; //(양바퀴 속도차 보정)

	Kp_tilt = 10;
	Kd_tilt = 0;//0.0000000001;
	Ki_tilt = 1;//0.0000001;



	object_theta = 0.0;
	err_theta = 0.0;
	prev_err_theta = 0.0;

	object_vel = 0.0;
	err_vel = 0.0;
	prev_err_vel = 0.0;

	//회전반경 초기화
	r_center = 0.0;
	r_lwheel = 0.0;
	r_rwheel = 0.0;

	//각속도 초기화	
	omega_lwheel = 0.0;
	omega_rwheel = 0.0;

	//선속도 초기화
	vel_lwheel = 0.0;
	vel_rwheel = 0.0;
	vel_center = 0.0;

	pi = 3.14159265;
	yaw_rate = 0.0;
	yaw_vel = 0.0;

	prev_err_yaw_vel = 0.0;
	err_yaw_vel = 0.0;
	object_yaw_vel = 0.0;

	rad_wheel = 0.205; //바퀴반지름[m]

	object_tilt = 0.0; //틸팅 모션은 rad (일단 나중에 바꾸더라도)
	prev_err_tilt = 0.0;
	err_tilt = 0.0;


	abs_tilt = 0.0;

	vel_controller = 0.0;
	pitch_controller = 0.0;
	tilt_controller = 0.0;

	yaw_gyro = 0.0;
	roll_gyro = 0.0;
	vel_err_sum = 0.0;

	err_lvel = 0.0;
	err_rvel = 0.0;
	prev_err_lvel = 0.0;
	prev_err_rvel = 0.0;

	object_lvel = 0.0;
	object_rvel = 0.0;

	lvel_err_sum = 0.0;
	rvel_err_sum = 0.0;

	lvel_controller = 0.0;
	rvel_controller = 0.0;

	lsum = 0.0;
	rsum = 0.0;
	pitch_err_sum = 0.0;
	tilt_err_sum = 0.0;
	pitch_theta = 0.0;

	err_rrvel = 0.0;
	err_llvel = 0.0;
	prev_yaw = 0.0;
	yaw_r = 0.0;

}

void controlsegway::update(const rTime& t)
{
	rControlAlgorithm::update(t);
}

void controlsegway::setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0)
{
}

void controlsegway::setPeriod(const rTime& dT)
{
}

void controlsegway::_readDevices()
{
	if (_gyro != INVALID_RHANDLE)
		readDeviceValue(_gyro, &_gyro_buf, 3 * sizeof(float));
	if (_enc_tilt != INVALID_RHANDLE)
		readDeviceValue(_enc_tilt, &_tilt_theta, sizeof(float));
	if (_tacho_lwheel != INVALID_RHANDLE)
		readDeviceValue(_tacho_lwheel, &_lwheel_vel_out, sizeof(float));
	if (_tacho_rwheel != INVALID_RHANDLE)
		readDeviceValue(_tacho_rwheel, &_rwheel_vel_out, sizeof(float));
	// htransform 단위[m], twist 단위[m/s]
}

void controlsegway::_writeDevices()
{
	if (_lmotor != INVALID_RHANDLE)
		writeDeviceValue(_lmotor, &_lWheel_des, sizeof(float));
	if (_rmotor != INVALID_RHANDLE)
		writeDeviceValue(_rmotor, &_rWheel_des, sizeof(float));
	if (_tmotor != INVALID_RHANDLE)
		writeDeviceValue(_tmotor, &_tmotor_des, sizeof(float));
}

void controlsegway::_reflect()
{
}

void controlsegway::_compute(const double& t)
{
}

void controlsegway::_estimate()
{

	omega_lwheel = _lwheel_vel_out;
	omega_rwheel = _rwheel_vel_out; //바퀴 각속도 rad/s (sensor)
	tilt_theta = _tilt_theta * 180 / pi; //tilting angle deg (sensor)
	pitch_theta = (_gyro, _gyro_buf[1]) * 180 / pi; //pitch angle deg (sensor)
	printf("%lf", pitch_theta);
	roll_gyro = (_gyro, _gyro_buf[2]) * 180 / pi;

	r_center = abs(vel_center / yaw_rate);
	abs_tilt = 0.9911 / (r_center + 0.7411);



	if (r_center > 2)//steering model
	{
		if (yaw_rate > 0.01)//좌선회
		{
			object_tilt = abs_tilt;
			object_rvel = (1.001 - 0.3993 * abs_tilt + 0.1793 * pow(abs_tilt, 2) + -0.5946 * pow(abs_tilt, 3)) * object_vel;
			object_lvel = (0.9986 + 0.4192 * abs_tilt - 0.3175 * pow(abs_tilt, 2) + 0.8723 * pow(abs_tilt, 3)) * object_vel;
		}

		else if (yaw_rate < -0.01)//우선회
		{
			object_tilt = -(abs_tilt);
			object_lvel = (1.001 - 0.3993 * abs_tilt + 0.1793 * pow(abs_tilt, 2) + -0.5946 * pow(abs_tilt, 3)) *object_vel;
			object_rvel = (0.9986 + 0.4192 * abs_tilt - 0.3175 * pow(abs_tilt, 2) + 0.8723 * pow(abs_tilt, 3)) *object_vel;
		}
		else// 전후진//속도에 대한 pd도 하기 (피치와 속도게인 이름 바꿔서)
		{

			object_tilt = 0;
			object_lvel = object_vel;
			object_rvel = object_vel;
		}
	}
	else //segway model
	{
		if (yaw_rate > 0.01)//좌선회
		{
			object_tilt = 0;
			object_lvel = object_vel*((r_center - 0.325) / r_center);
			object_rvel = object_vel*((r_center + 0.325) / r_center);
		}

		else if (yaw_rate < -0.01)//우선회
		{
			object_tilt = 0.0;
			object_rvel = object_vel*((r_center - 0.325) / r_center);
			object_lvel = object_vel*((r_center + 0.325) / r_center);
		}
		else // 전후진//속도에 대한 pd도 하기 (피치와 속도게인 이름 바꿔서)
		{
			object_tilt = 0.0;
			object_lvel = object_vel;
			object_rvel = object_vel;
		}

	}



	vel_lwheel = (omega_lwheel*rad_wheel);
	vel_rwheel = (omega_rwheel*rad_wheel);//바퀴 속도 계산

	vel_center = (vel_lwheel + vel_rwheel) / 2;




	//속도 제어기
	prev_err_lvel = err_lvel;
	err_lvel = object_lvel - vel_center; //왼바퀴 에러
	err_llvel = object_lvel - vel_lwheel;
	lvel_err_sum += (err_lvel + prev_err_lvel) * dt / 2;
	//왼바퀴 속도제어기PID
	lvel_controller = (Kp_vel*err_lvel + (Kd_vel * (err_lvel - prev_err_lvel) / dt + Ki_vel) * lvel_err_sum)+K_diff * err_llvel;


	prev_err_rvel = err_rvel;
	err_rvel = object_rvel - vel_center; //오른바퀴 에러
	err_rrvel = object_rvel - vel_rwheel;
	rvel_err_sum += (err_rvel + prev_err_rvel) * dt / 2;
	//오른바퀴 속도제어기PID
	rvel_controller = (Kp_vel*err_rvel + (Kd_vel * (err_rvel - prev_err_rvel) / dt + Ki_vel) * rvel_err_sum)+ K_diff * err_rrvel;


	/*
	prev_err_lvel = err_lvel;
	err_lvel = object_lvel - vel_lwheel; //왼바퀴 에러
	lvel_err_sum += (err_lvel + prev_err_lvel) * dt / 2;
	//왼바퀴 속도제어기PID
	lvel_controller = (Kp_vel*err_lvel + (Kd_vel * (err_lvel - prev_err_lvel) / dt + Ki_vel) * lvel_err_sum);


	prev_err_rvel = err_rvel;
	err_rvel = object_rvel - vel_rwheel; //오른바퀴 에러
	rvel_err_sum += (err_rvel + prev_err_rvel) * dt / 2;
	//오른바퀴 속도제어기PID
	rvel_controller = (Kp_vel*err_rvel + (Kd_vel * (err_rvel - prev_err_rvel) / dt + Ki_vel) * rvel_err_sum);
	*/

	//자세 제어기
	prev_err_theta = err_theta;
	err_theta = object_theta - pitch_theta; //자세에러
	pitch_err_sum += (err_theta + prev_err_theta)*dt / 2;
	//자세 제어기PID
	pitch_controller = (Kp_pitch * err_theta + (Kd_pitch * (err_theta - prev_err_theta) / dt) + Ki_pitch *pitch_err_sum);


	prev_yaw = yaw_gyro;
	yaw_gyro = (_gyro, _gyro_buf[0]) * 180 / pi;

	yaw_r = (-prev_yaw + yaw_gyro) / dt;


	//tilting motion 제어기
	prev_err_tilt = err_tilt;
	err_tilt = object_tilt - tilt_theta; //틸팅각에러
	tilt_err_sum += (err_tilt + prev_err_tilt) *dt / 2;

	//tilting제어기
	tilt_controller = (Kp_tilt * err_tilt + (Kd_tilt * (err_tilt - prev_err_tilt) / dt) + Ki_tilt * tilt_err_sum);

	//pid controller calculate

	lsum = lvel_controller*1.5 + pitch_controller*2;
	rsum = rvel_controller*1.5 + pitch_controller*2;


	_tmotor_des = tilt_controller;
	_rWheel_des = rsum;
	_lWheel_des = lsum;



	printf("yaw:,%f, yaw_rate:,%f, torque:,%f,%f,\n", yaw_gyro, yaw_r, _lWheel_des, _rWheel_des);
	//printf("err_lvel:,%f, lvel_err_sum:,%f,object_lvel:,%f,\n", err_lvel, lvel_err_sum, object_lvel);
	//printf("err_rvel:,%f, rvel_err_sum:,%f,object_rvel:,%f,\n", err_rvel, rvel_err_sum, object_rvel);
	//printf("tilt:%f, object_tilt:%f, vel:,%f,\n", tilt_theta, object_tilt , vel_center);
	//printf("r:%f, ref_lvel:%f,ref_rvel:,%f,\n\n",r_center, object_lvel, object_rvel);
	//printf("pitch:%f, lvel:%f,rvel:,%f,\n\n", err_theta, vel_lwheel ,vel_rwheel);
	//printf("l_input:%f, r_input:%f,pitch_input:%f,\n\n", lvel_controller*2, rvel_controller*2, pitch_controller);

}


int controlsegway::command(const short& cmd, const int& arg)
{
	
	printf("a");

	return 0;
}

void controlsegway::datanames(vector<string_type>& names, int channel)
{
}

void controlsegway::collect(vector<double>& data, int channel)
{
}

void controlsegway::onSetInterestFrame(const TCHAR* name, const HTransform& T)
{
}

rControlAlgorithm* CreateControlAlgorithm(rDC& rdc)
{
	return new controlsegway(rdc);
}
