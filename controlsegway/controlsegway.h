#ifndef __CONTROLSEGWAY_H__
#define __CONTROLSEGWAY_H__

#include <list>
#include "rControlAlgorithm/rControlAlgorithm.h"

#ifdef _USE_RCONTROLALGORITHM_EX_
class REXPORT controlsegway : public rControlAlgorithmEx
#else
class REXPORT controlsegway : public rControlAlgorithm
#endif
{
public:
	controlsegway(rDC rdc);
	~controlsegway();

	virtual void init(int mode = 0);
	virtual void update(const rTime& t);
	virtual void setNominalSystem(const TCHAR* path, const TCHAR* aml, const HTransform& T0, const dVector& q0);
	virtual void setPeriod(const rTime& dT);
	virtual int command(const short& cmd, const int& arg = 0);
	virtual void datanames(vector<string_type>& names, int channel = -1);
	virtual void collect(vector<double>& data, int channel = -1);
	virtual void onSetInterestFrame(const TCHAR* name, const HTransform& T);

private:
	virtual void _estimate();
	virtual void _readDevices();
	virtual void _writeDevices();

	virtual void _reflect();
	virtual void _compute(const rTime& t);

	void _arrangeJointDevices();


private:

	//������ devices
	rHANDLE	_lmotor;
	rHANDLE	_rmotor;
	rHANDLE	_tmotor;

	//���� devices
	rHANDLE _gyro;
	rHANDLE _enc_tilt;
	rHANDLE _tacho_lwheel;
	rHANDLE _tacho_rwheel;

	//������ devices�� �Է°� (_actuator_buf�� _wrench�� �Է°�)
	float	_lWheel_des;
	float	_rWheel_des;
	float	_tmotor_des;
	float	_tmotor_des_deg;
	float	_virtualmotor_bar_des;


	//���� devices�� ��°�
	float	_gyro_buf[3];

	float	tilt_theta;
	float	_lwheel_vel_out;
	float	_rwheel_vel_out;


	//PD��� ���� ����
	float	Kp_pitch;
	float	Kd_pitch;
	float	dt;

	float	object_theta;
	float	err_theta;
	float	prev_err_theta;

	//ȸ���ݰ�
	float	r_center;
	float	r_lwheel;
	float	r_rwheel;

	//����� ���ӵ�
	float	omega_rwheel;
	float	omega_lwheel;

	//����� ���ӵ�
	float	vel_lwheel;
	float	vel_rwheel;

	//body �ӵ� (����� ���ӵ� ���)
	float	vel_center;

	//���� ������
	float	rad_wheel;

	//�ӵ� �����
	float	object_vel;
	float	err_vel;
	float	prev_err_vel;
	float	Kp_vel;
	float	Kd_vel;
	float	Ki_vel;
	float	dt_vel;
	float	del_err_vel;
	float	pi;

	float	yaw_rate;
	float	yaw_vel;

	float	Kp_yaw_vel;
	float	Kd_yaw_vel;
	float	prev_err_yaw_vel;
	float	err_yaw_vel;
	float	object_yaw_vel;
	float	r_wheel_center;



	float	object_tilt;
	float	err_tilt;
	float	prev_err_tilt;
	float	Kp_tilt;
	float	Kd_tilt;
	float	_tilt_theta;

	float	vel_err_sum;
	float	tilt_err_sum;



	float	object_lvel;
	float	object_rvel;
	float	err_rvel;
	float	err_lvel;
	float	prev_err_rvel;
	float	prev_err_lvel;

	//controller
	float	abs_tilt;

	float	Ki_pitch;
	float	Ki_tilt;
	float	vel_controller;
	float	pitch_controller;
	float	tilt_controller;
	float	yaw_gyro;
	float	roll_gyro;


	float	lvel_err_sum;
	float	rvel_err_sum;
	float	lvel_controller;
	float	rvel_controller;

	float	lsum;
	float	rsum;
	float	pitch_err_sum;
	float	pitch_theta;
	float	K_diff;
	float	err_rrvel;
	float	err_llvel;
	float	prev_yaw;
	float	yaw_r;

};
#endif