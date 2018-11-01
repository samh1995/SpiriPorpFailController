/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <systemlib/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#define MIN_TAKEOFF_THRUST    0.2f
#define TPA_RATE_LOWER_LIMIT 0.05f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

//custom
/*#define Kap 110.0f
#define Kad 20.0f
#define Kpp 0.08f
#define Kpd 0.10f
#define Khp 5.0f
#define Khi 0.50f
#define Kv 3.0f*/
#define Kaero 0.0f
#define uref 0.0f
#define Omega_max 5.0f
#define Omega_dot_rise 50.0f
#define Omega_dot_fall 10.0f
#define t_peak 0.2f
#define man_kind 5
#define man_kind2 6


#define Ix 0.012275f //Moment of Inertia about x (kg m^2)
#define Iy 0.012661f //Moment of Inertia about y (kg m^2)
#define Iz 0.023851f //Moment of Inertia about z (kg m^2)
#define Ixy -0.00007295f
#define Iyz 0.00001187f
#define Ixz 0.00001589f
#define kt 0.00000010296f //Thruster Constant (N/(RPM)^2) 0.000000087f
#define kq 0.000000010296f  //Thruster Constant (N/(RPM)^2) 0.0000000087f
#define l 0.23f // Arm Length (m)
#define w_max 8534.86f
#define m 1.023f //Mass (kg)
#define F_aero0 0.0f //F_aero = F_aero2*v^2 + F_aero1*v + F_aero0
#define F_aero1 0.0f
#define F_aero2 0.0f

using namespace matrix;


int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude and rate controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) or rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has two loops: a P loop for angular error and a PID loop for angular rate error.

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

### Implementation
To reduce control latency, the module directly polls on the gyro topic published by the IMU driver.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :
	ModuleParams(nullptr),
	//custom
	_aerobatics_variables_pub(nullptr),
	_aerobatics_variables2_pub(nullptr),
	_aerobatics_variables3_pub(nullptr),
	_debug_pub(nullptr),

	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),


	_lp_filters_d{
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f},
	{initial_update_rate_hz, 50.f}} // will be initialized correctly when params are loaded
{
	for (uint8_t i = 0; i < MAX_GYRO_COUNT; i++) {
		_sensor_gyro_sub[i] = -1;
	}

	_vehicle_status.is_rotary_wing = true;

	/* initialize quaternions in messages to be valid */
	_v_att.q[0] = 1.f;
	_v_att_sp.q_d[0] = 1.f;
	_v_pos.x = 0.f;
	_v_pos.y = 0.f;
	_v_pos.z = 0.f;
	_v_pos.vx = 0.f;
	_v_pos.vy = 0.f;
	_v_pos.vz = 0.f;


	_rates_prev.zero();
	_rates_prev_filtered.zero();
	_rates_sp.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	/* initialize thermal corrections as we might not immediately get a topic update (only non-zero values) */
	for (unsigned i = 0; i < 3; i++) {
		// used scale factors to unity
		_sensor_correction.gyro_scale_0[i] = 1.0f;
		_sensor_correction.gyro_scale_1[i] = 1.0f;
		_sensor_correction.gyro_scale_2[i] = 1.0f;
	}

	parameters_updated();
	memset(&_aerobatics_variables, 0, sizeof(_aerobatics_variables));

}

void
MulticopterAttitudeControl::parameters_updated()
{
	/* Store some of the parameters in a more convenient way & precompute often-used values */

	/* roll gains */
	_attitude_p(0) = _roll_p.get();
	_rate_p(0) = _roll_rate_p.get();
	_rate_i(0) = _roll_rate_i.get();
	_rate_int_lim(0) = _roll_rate_integ_lim.get();
	_rate_d(0) = _roll_rate_d.get();
	_rate_ff(0) = _roll_rate_ff.get();

	/* pitch gains */
	_attitude_p(1) = _pitch_p.get();
	_rate_p(1) = _pitch_rate_p.get();
	_rate_i(1) = _pitch_rate_i.get();
	_rate_int_lim(1) = _pitch_rate_integ_lim.get();
	_rate_d(1) = _pitch_rate_d.get();
	_rate_ff(1) = _pitch_rate_ff.get();

	/* yaw gains */
	_attitude_p(2) = _yaw_p.get();
	_rate_p(2) = _yaw_rate_p.get();
	_rate_i(2) = _yaw_rate_i.get();
	_rate_int_lim(2) = _yaw_rate_integ_lim.get();
	_rate_d(2) = _yaw_rate_d.get();
	_rate_ff(2) = _yaw_rate_ff.get();

	if (fabsf(_lp_filters_d[0].get_cutoff_freq() - _d_term_cutoff_freq.get()) > 0.01f) {
		_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
		_lp_filters_d[0].reset(_rates_prev(0));
		_lp_filters_d[1].reset(_rates_prev(1));
		_lp_filters_d[2].reset(_rates_prev(2));
	}

	/* angular rate limits */
	_mc_rate_max(0) = math::radians(_roll_rate_max.get());
	_mc_rate_max(1) = math::radians(_pitch_rate_max.get());
	_mc_rate_max(2) = math::radians(_yaw_rate_max.get());

	/* auto angular rate limits */
	_auto_rate_max(0) = math::radians(_roll_rate_max.get());
	_auto_rate_max(1) = math::radians(_pitch_rate_max.get());
	_auto_rate_max(2) = math::radians(_yaw_auto_max.get());

	/* manual rate control acro mode rate limits and expo */
	_acro_rate_max(0) = math::radians(_acro_roll_max.get());
	_acro_rate_max(1) = math::radians(_acro_pitch_max.get());
	_acro_rate_max(2) = math::radians(_acro_yaw_max.get());

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	/* get transformation matrix from sensor/board to body frame */
	_board_rotation = get_rot_matrix((enum Rotation)_board_rotation_param.get());

	/* fine tune the rotation */
	Dcmf board_rotation_offset(Eulerf(
			M_DEG_TO_RAD_F * _board_offset_x.get(),
			M_DEG_TO_RAD_F * _board_offset_y.get(),
			M_DEG_TO_RAD_F * _board_offset_z.get()));
	_board_rotation = board_rotation_offset * _board_rotation;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		updateParams();
		parameters_updated();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (_rates_sp_id == nullptr) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		multirotor_motor_limits_s motor_limits = {};
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &motor_limits);

		_saturation_status.value = motor_limits.saturation_status;
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

void
MulticopterAttitudeControl::vehicle_local_position_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_v_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _v_pos_sub, &_v_pos);
	}
}

void
MulticopterAttitudeControl::sensor_correction_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_correction_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_correction), _sensor_correction_sub, &_sensor_correction);
	}

	/* update the latest gyro selection */
	if (_sensor_correction.selected_gyro_instance < _gyro_count) {
		_selected_gyro = _sensor_correction.selected_gyro_instance;
	}
}

void
MulticopterAttitudeControl::sensor_bias_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_sensor_bias_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sensor_bias), _sensor_bias_sub, &_sensor_bias);
	}

}


/**
 * Controller.
 * Input: States, Reference Motion
 * Output: Actuator Commands
 */

void 
MulticopterAttitudeControl::controller(float dt)
{

	//Constants and Aircraft Properties
	//float ro = 1.225f; //Air Density (kg/m^3)
	Kap = 110;
	Kad = 20;
	Kpp = 0.08;
	Kpd = 0.1;
	Khp = 5.0f;
	Khi = 0.5f;
	Kv = 3.0f;
	wbbar2=1.28618f;
	nbar2=0.04962f;

//	float T_max = kt * w_max * w_max;
	//float T_max = 7.254f;
	float T_max = 7.514f;
	float T_min = 0f;
	float F_max = 4.0f * T_max;
	float F_min = 4.0f * T_min;
	float F_b;
	math::Vector<3> F_hat_r(0.0f,0.0f,-1.0f); // direction of body-fixed thrust
	math::Quaternion q_des;
	math::Matrix<3, 3> C_bi= q.to_dcm().transposed(); //Rotation Matrix from inertial to body frame
	math::Matrix<3, 3> C_ri= q_ref.to_dcm().transposed(); //Rotation Matrix from inertial to ref frame

	if (int(_manual_control_sp.aux1) == -1) {

	//Manual Setpoints
	float phi_des = _manual_control_sp.y * (15.0f * PI/180.0f);
	float theta_des = -_manual_control_sp.x * (15.0f * PI/180.0f);
	float psi_des = psi + _manual_control_sp.r * (15.0f * PI/180.0f); //no yaw feedback control, only yaw rate
	if (psi_des>PI){psi_des = psi_des - 2.0f*PI;}
	if (psi_des<-PI){psi_des = psi_des + 2.0f*PI;}

	q_des.from_euler(phi_des, theta_des, psi_des);
	omega_ref_r.zero();

	//Thrust Controller----------------------------------------------------------------------------------------------------------------------------------
  	F_b = _manual_control_sp.z * F_max;

	//Attitude Controller------------------------------------------------------------------------------------------------------------------------------------
 	//if ((q+q_des).length() < (q-q_des).length()){q_des=-q_des;} //This ensures delta_q results in error angles less than 180 deg
 	math::Quaternion delta_q = q.conjugated() * q_des; //Error Quaternion
 	if (delta_q(0)<-1.0f){delta_q(0) = -1.0f;} //if slightly larger than |1| acos will give problems
 	if (delta_q(0)>1.0f){delta_q(0) = 1.0f;}
	if (delta_q(0) < 0.0f){delta_q = -delta_q;}
	math::Vector<3> E_b;
	if (delta_q.imag().length() < 0.000001f){
		E_b = math::Vector<3>(0.0f, 0.0f, 0.0f);
	}
	else {
		E_b = math::Vector<3>(2.0f*acosf(delta_q(0))*delta_q(1)/delta_q.imag().length() , 2.0f*acosf(delta_q(0))*delta_q(2)/delta_q.imag().length() , 2.0f*acosf(delta_q(0))*delta_q(3)/delta_q.imag().length());
	}

	// Modify Kap and Kad for yaw
	math::Matrix <3, 3> Kp_;
	Kp_.zero();
	Kp_(0,0) = Kap * 1.0f;
	Kp_(1,1) = Kap * 1.0f;
	Kp_(2,2) = Kap * 0.1f;

	math::Matrix <3, 3> Kd_;
	Kd_.zero();
	Kd_(0,0) = Kad * 1.0f;
	Kd_(1,1) = Kad * 1.0f;
	Kd_(2,2) = Kad * 0.1f;

	float I[3][3] = {{Ix, Ixy, Ixz}, {Ixy, Iy, Iyz}, {Ixz, Iyz, Iz}};
	math::Matrix<3, 3> I_matrix(I);

	//math::Vector<3> M_b = I_matrix * ( E_b * Kap + (C_bi * C_ri.transposed() * omega_ref_r - omega_b) * Kad);
	math::Vector<3> E_temp = Kp_ * E_b + Kd_ * (C_bi * C_ri.transposed() * omega_ref_r - omega_b);
	math::Vector<3> M_b = I_matrix * E_temp;


  	//Mixer----------------------------------------------------------------------------------------------------------------------------------------------------

	//F_b = math::constrain(F_b, 0.0f, F_max); 
	F_b = math::constrain(F_b, F_min, F_max); 

    /*float T1 =  M_b(1)/(2.0f*l) + M_b(2)*kt/(4.0f*kq) + F_b/4.0f;
    float T2 = -M_b(0)/(2.0f*l) - M_b(2)*kt/(4.0f*kq) + F_b/4.0f;
    float T3 = -M_b(1)/(2.0f*l) + M_b(2)*kt/(4.0f*kq) + F_b/4.0f;
    float T4 =  M_b(0)/(2.0f*l) - M_b(2)*kt/(4.0f*kq) + F_b/4.0f;*/
    float kt_kq = 56.0f;
	float temp[4][4] = {{0.25f, -.3536f/l, .3536f/l, .25f*kt_kq} , {0.25f, .3536f/l, -.3536f/l, .25f*kt_kq}, {0.25f, .3536f/l, .3536f/l, -0.25f*kt_kq}, {0.25f, -.3536f/l, -.3536f/l, -0.25f*kt_kq}};
	math::Matrix<4, 4> tothrust(temp);
	math::Vector<4> FM_b(F_b, M_b(0), M_b(1), M_b(2));
    math::Vector<4> T = tothrust*FM_b;
    //math::Vector<4> Omega;
    math::Vector<4> cmd;
    math::Vector<4> PWM;

   	for (int i = 0; i < 4; i++) {
   		T(i) = math::constrain(T(i), 0.0f, T_max);
		//T(i) = math::constrain(T(i),T_min, T_max);

   		PWM(i) = -40.968f + 0.0202511f * sqrt(9.876f * powf(10.0f,8.0f) * T(Si) + 2.88196f * powf(10.0f,9.0f));

   		// Bidirectional Thrust
   		/*if( T(i) >= 0.0f) {
   			PWM(i) = 1051.78f + 2.17581f * sqrt(18384.0f * T(i) + 48181.0f);
   		}
   		else if (T(i) < 0.0f) {
   			PWM(i) = 1721.69f - 0.318066f * sqrt(646153.0f - 628800.0f * T(i));
   		}*/
   		cmd(i) = (PWM(i)-1000.0f)/1000.0f;
  		cmd(i) = math::constrain(cmd(i),0.0f,1.0f);

   		//Omega(i) = sqrt(T(i)/kt);
  		//cmd(i) = 3.7658f*powf(10.0f,-9.0f)*Omega(i)*Omega(i) + 6.7411f*powf(10.0f,-5.0f)*Omega(i) - .0812f +0.0f ;
   	} 



  	//Send Signals---------------------------------------------------------------------------------------------------------------------------------------
  	_actuators.control[0] = cmd(0);
	_actuators.control[1] = cmd(1);
	_actuators.control[2] = cmd(2);
	_actuators.control[3] = cmd(3);
	_actuators.timestamp = hrt_absolute_time();
	_actuators.timestamp_sample = _sensor_gyro.timestamp;
}

else {
								////////////////////// SAM CONTROLLER///////////////////////////////////////////////

		math::Vector<3> desrdposn;
		desrdposn(0)= -_manual_control_sp.x * (15.0f * PI/180.0f);
		desrdposn(1)=_manual_control_sp.y * (15.0f * PI/180.0f);
		desrdposn(2)=3.0f;

		math::Vector<3> desiredVinertial;
		desiredVinertial(0)=0.0f;
		desiredVinertial(1)=0.0f;
		desiredVinertial(2)=0.0f;

		float damping_ratio=0.7;
		float nat_freq=1.0;
		float m=1.095;


		math::Vector<3> nbar(0.0f,0.0496243584044f,0.9987682823025f);
		math::Vector<3> gravity(0.0f, 0.0f, 9.81f);
		math::Vector<3> errouter_d(p_i(0)-desrdposn(0),p_i(1)-desrdposn(1),p_i(2)-desrdposn(2));
		// 		math::Vector<3> errouter_d(desrdposn(0),desrdposn(1),desrdposn(2));
		math::Vector<3> errouter_d_dot(v_i(0)-desiredVinertial(0),v_i(1)-desiredVinertial(1),v_i(2)-desiredVinertial(2));
		math::Vector<3> desiredaccel;
		desiredaccel=-2*damping_ratio*nat_freq*errouter_d_dot-pow(nat_freq,2)*errouter_d;

		// constant K for now, add lqr solver later
		math::Matrix<2,4> K;
		K(0,0)=0.0f;
		K(0,1)=1.2f;
		K(0,2)=-3.5478f;
		K(0,3)=-4.1729f;
		K(1,0)=1.2f;
		K(1,1)=0.0f;
		K(1,2)=-4.1729f;
		K(1,3)=3.5478f;

		math::Vector<3> f_vec;

		f_vec=(m/nbar(2))*(desiredaccel-gravity);
		float f_total; //f_total.normalize();
		f_total=sqrt(pow(f_vec(0),2)+pow(f_vec(1),2)+pow(f_vec(2),2));


		/*math::Quaternion quat(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
		
		math::Matrix<3,3> Rotationmatrix;
		Rotationmatrix(0,0)=(pow(quat(0),2)+pow(quat(1),2)-pow(quat(2),2)-pow(quat(3),2));
		Rotationmatrix(1,0)=2*(quat(1)*quat(2)+quat(0)*quat(3));
		Rotationmatrix(2,0)=2*(quat(1)*quat(3)-quat(0)*quat(2));

		Rotationmatrix(0,1)=2*(quat(1)*quat(2)-quat(0)*quat(3));
		Rotationmatrix(1,1)=(pow(quat(0),2)-pow(quat(1),2)+pow(quat(2),2)-pow(quat(3),2));
		Rotationmatrix(2,1)=2*(quat(2)*quat(3)+quat(0)*quat(1));

		Rotationmatrix(0,2)=2*(quat(1)*quat(3)+quat(0)*quat(2));
		Rotationmatrix(1,2)=2*(quat(2)*quat(3)-quat(0)*quat(1));
		Rotationmatrix(2,2)=(pow(quat(0),2)-pow(quat(1),2)-pow(quat(2),2)+pow(quat(3),2));*/
		

		//Rotation matrix from inertial to body to dand coordiantes:
		math::Matrix<3, 3> R_I2F= q.to_dcm().transposed(); //Rotation Matrix from inertial to body frame
		math::Matrix<3,3> R_F2D;

		R_F2D.zero();
		R_F2D(0,0)=sqrt(2.0)/2.0;
		R_F2D(0,1)=sqrt(2.0)/2.0;
		R_F2D(1,0)=sqrt(2.0)/2.0;
		R_F2D(1,1)=-1.0*sqrt(2.0)/2.0;
		R_F2D(2,2)=-1.0;
		
		math::Matrix<3,3> R_I2D;
		R_I2D=R_F2D*R_I2F;     // if it doesnt work use library math 


		//attitude Controller
		math::Vector<3> n_desired;
		n_desired=m/nbar(2)*R_I2D*(desiredaccel-gravity)/f_total;
		
		// these are the desired body rates
		math::Vector<3> bodyRates_tmp(omega_b(0),omega_b(1),omega_b(2));
		math::Vector<3> bodyRates;
		bodyRates=R_F2D*bodyRates_tmp;

		/*_recovery_control.bodyRatesDesired[0] = bodyRates(0);
		_recovery_control.bodyRatesDesired[1] = bodyRates(1);
		_recovery_control.bodyRatesDesired[2] = bodyRates(2);*/

		math::Vector<4> dandgains(bodyRates[0],bodyRates[1]-wbbar2, n_desired(0), n_desired(1)-nbar2);
		math::Vector<2> dandinput;
		dandinput=-K*dandgains;

		// solving for froces converting to ind motor thrusts
		
		float temp1[3][3] = {{0.5f, -0.5f, -0.5f} , {0.0f , 0.0f , 1.0f}, {0.5f, 0.5f, 0.5f}};
		math::Matrix<3, 3> mixermatrixG(temp1);

		math::Vector<3> dandinputforces(f_total, dandinput(0), dandinput(1)+2.151f);
		math::Vector<3> tempF_b=mixermatrixG*dandinputforces;
		math::Vector<4> T(tempF_b(0),tempF_b(1),tempF_b(2),0);
// End of My controller
		// Sams Mixer
  		math::Vector<4> cmd;
    	math::Vector<4> PWM;
    	for (int i = 0; i < 4; i++) {
			T(i) = math::constrain(T(i), 0.0f, T_max);
			PWM(i) = -40.968f + 0.0202511f * sqrt(9.876f * powf(10.0f,8.0f) * T(Si) + 2.88196f * powf(10.0f,9.0f));
			cmd(i) = (PWM(i)-1000.0f)/1000.0f;
  			cmd(i) = math::constrain(cmd(i),0.0f,1.0f);
  		}


  	//Send Signals---------------------------------------------------------------------------------------------------------------------------------------
  	_actuators.control[0] = cmd(0);
	_actuators.control[1] = cmd(1);
	_actuators.control[2] = cmd(2);
	_actuators.control[3] = cmd(3);
	_actuators.timestamp = hrt_absolute_time();
	_actuators.timestamp_sample = _sensor_gyro.timestamp;

}





	//Send parameters to data logger
    _aerobatics_variables.Quaternion_Desired[0] = q_des(0);
    _aerobatics_variables.Quaternion_Desired[1] = q_des(1);
    _aerobatics_variables.Quaternion_Desired[2] = q_des(2);
    _aerobatics_variables.Quaternion_Desired[3] = q_des(3);
    _aerobatics_variables.Position_Reference[0] = p_ref_i(0);
    _aerobatics_variables.Position_Reference[1] = p_ref_i(1);
    _aerobatics_variables.Position_Reference[2] = p_ref_i(2);
    _aerobatics_variables.Slipstream = 0;
    _aerobatics_variables.Speed_Reference = uref;
    _aerobatics_variables.Quaternion_Reference[0] = q_ref(0);
    _aerobatics_variables.Quaternion_Reference[1] = q_ref(1);
    _aerobatics_variables.Quaternion_Reference[2] = q_ref(2);
    _aerobatics_variables.Quaternion_Reference[3] = q_ref(3);
	_aerobatics_variables.timestamp = hrt_absolute_time();

	// Log individual rotor thrusts and total force and moments
	_aerobatics_variables.Thrust[0] = T(0);
	_aerobatics_variables.Thrust[1] = T(1);
	_aerobatics_variables.Thrust[2] = T(2);
	_aerobatics_variables.Thrust[3] = T(3);

	_aerobatics_variables3.F_b = F_b;
	_aerobatics_variables3.M_x = M_b(0);
	_aerobatics_variables3.M_y = M_b(1);
	_aerobatics_variables3.M_z = M_b(2);


    _aerobatics_variables2.kpp = Kpp;
	_aerobatics_variables2.kpd = Kpd;
	_aerobatics_variables2.kpi = 0;
	_aerobatics_variables2.kap = Kap;
	_aerobatics_variables2.kad = Kad;
	_aerobatics_variables2.kai = 0;
	_aerobatics_variables2.ku = Kv;
	_aerobatics_variables2.khp = Khp;
	_aerobatics_variables2.khd = 0;
	_aerobatics_variables2.khi = Khi;
	_aerobatics_variables2.semi_auto = 0;


    _aerobatics_variables3.ail = cmd(0);
    _aerobatics_variables3.elev = cmd(1);
    _aerobatics_variables3.rud = cmd(2);
    _aerobatics_variables3.thr = cmd(3);
    _aerobatics_variables3.ex = E_b(0);
    _aerobatics_variables3.ey = E_b(1);
    _aerobatics_variables3.ez = E_b(2);
    _aerobatics_variables3.kdc = Kaero;
	_aerobatics_variables3.kdv = omega_ref_r(0);

	_debug.floats[0] = omega_ref_r(0);
    _debug.floats[1] = omega_ref_r(1);
    _debug.floats[2] = omega_ref_r(2);
    _debug.floats[3] = v_ref_r(0);
    _debug.floats[4] = v_ref_r(1);
    _debug.floats[5] = v_ref_r(2);
    //_debug.floats[6] = outputs[3];
    //_debug.floats[7] = Vs_filt;
    //_debug.floats[8] = dt;

}

/**
 * Maneuver_Generator.
 * Input: States
 * Output: Reference Motion Variables
 */

void 
MulticopterAttitudeControl::maneuver_generator()
{


	//Define maneuver switch
	if (maneuver_type-maneuver_type_old!=0){maneuver_switch = true;}else{maneuver_switch = false;}
	/*
	1)hover 
	2)rolling flip
	*/

	if (maneuver_type==1)
	{
		if (maneuver_switch == true)
		{
			p_ref_i = p_i;
			v_ref_r.zero();
			q_ref.from_euler(0.0f, 0.0f, psi_ref);
			omega_ref_r.zero();
		}

	}

	if (maneuver_type==2)
	{
		if (maneuver_switch == true)
		{
			p_ref_i = p_i;
			v_ref_r.zero();
			start_time = hrt_absolute_time();
			t_1 = Omega_max/Omega_dot_rise;
			t_2 = (4.0f * PI/Omega_max + (Omega_max / Omega_dot_rise - Omega_max / Omega_dot_fall)) / 2.0f;
			t_3 = Omega_max/Omega_dot_fall + t_2;
		}
		float t = (hrt_absolute_time() - start_time) / 1000000.0f;

		if (t < t_1){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * powf(t,2.0f) / (2.0f * t_1), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * t / t_1, 0.0f, 0.0f);
		}
		else if (t < t_2){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * (t - t_1 / 2.0f), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max, 0.0f, 0.0f);
		}
		else if (t < t_3){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * (t - t_1 / 2.0f) - Omega_max * powf(t - t_2, 2.0f) / (2.0f * (t_3 - t_2)), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * (1.0f - (t-t_2)/(t_3-t_2)), 0.0f, 0.0f);
		}
		else{
			q_ref.from_euler(0.0f, 0.0f, psi_ref);
			omega_ref_r.zero();
		}

	}

	if (maneuver_type==5)//level to inverted
	{
		if (maneuver_switch == true)
		{
			p_ref_i = p_i;
			v_ref_r.zero();
			start_time = hrt_absolute_time();
			t_1 = Omega_max/Omega_dot_rise;
			t_2 = (2.0f * PI/Omega_max + (Omega_max / Omega_dot_rise - Omega_max / Omega_dot_fall)) / 2.0f;
			t_3 = Omega_max/Omega_dot_fall + t_2;
		}
		float t = (hrt_absolute_time() - start_time) / 1000000.0f;

		if (t < t_1){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * powf(t,2.0f) / (2.0f * t_1), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * t / t_1, 0.0f, 0.0f);
		}
		else if (t < t_2){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * (t - t_1 / 2.0f), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max, 0.0f, 0.0f);
		}
		else if (t < t_3){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(Omega_max * (t - t_1 / 2.0f) - Omega_max * powf(t - t_2, 2.0f) / (2.0f * (t_3 - t_2)), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * (1.0f - (t-t_2)/(t_3-t_2)), 0.0f, 0.0f);
		}
		else{
			q_ref.from_euler(PI, 0.0f, psi_ref);
			omega_ref_r.zero();
		}

	}

	if (maneuver_type==6)//inverted to level
	{
		if (maneuver_switch == true)
		{
			p_ref_i = p_i;
			v_ref_r.zero();
			start_time = hrt_absolute_time();
			t_1 = Omega_max/Omega_dot_rise;
			t_2 = (2.0f * PI/Omega_max + (Omega_max / Omega_dot_rise - Omega_max / Omega_dot_fall)) / 2.0f;
			t_3 = Omega_max/Omega_dot_fall + t_2;
		}
		float t = (hrt_absolute_time() - start_time) / 1000000.0f;

		if (t < t_1){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(PI + Omega_max * powf(t,2.0f) / (2.0f * t_1), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * t / t_1, 0.0f, 0.0f);
		}
		else if (t < t_2){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(PI + Omega_max * (t - t_1 / 2.0f), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max, 0.0f, 0.0f);
		}
		else if (t < t_3){
			Kpp = 0.0f;
			Kpd = 0.0f;
			q_ref.from_euler(PI + Omega_max * (t - t_1 / 2.0f) - Omega_max * powf(t - t_2, 2.0f) / (2.0f * (t_3 - t_2)), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>(Omega_max * (1.0f - (t-t_2)/(t_3-t_2)), 0.0f, 0.0f);
		}
		else{
			q_ref.from_euler(0.0f, 0.0f, psi_ref);
			omega_ref_r.zero();
		}

	}


	if (maneuver_type==3)
	{
		if (maneuver_switch == true)
		{
			p_ref_i = p_i;
			v_ref_r.zero();
			start_time = hrt_absolute_time();
			t_f = (4.0f * PI/Omega_max);
		}
		float t = (hrt_absolute_time() - start_time) / 1000000.0f;

		if (t < t_peak){
			q_ref.from_euler((Omega_max/2.0f) * ((-t_peak/PI)*cosf((t-t_peak/2.0f)*(PI/t_peak))+t), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>((Omega_max/2.0f) * (sinf((t-t_peak/2.0f)*(PI/t_peak))+1.0f), 0.0f, 0.0f);
		}
		else if (t < t_f){
			q_ref.from_euler((Omega_max/2.0f) * (((t_f-t_peak)/PI)*sinf((t-t_peak)*(PI/(t_f-t_peak)))+t), 0.0f, psi_ref);
			omega_ref_r = math::Vector<3>((Omega_max/2.0f) * (cosf((t-t_peak)*(PI/(t_f-t_peak)))+1.0f), 0.0f, 0.0f);
		}
		else{
			q_ref.from_euler(0.0f, 0.0f, psi_ref);
			omega_ref_r.zero();
		}

	}

	// Bidirectional Thrust
	if (maneuver_type == 4) {
		if (maneuver_switch == true) {
			p_ref_i = p_i;
			v_ref_r.zero();
			q_ref.from_euler(0.0f,0.0f,psi_ref);
			omega_ref_r.zero();
			start_time = hrt_absolute_time();

		}
		float t = (hrt_absolute_time() - start_time) / 1000000.0f;

		if (t < 0.25f) {
			a_ref = math::Vector<3>(0.0f,0.0f, -20.2493f);
			v_ref_r = a_ref * t;

			Kap = 110.0f;
			Kad = 20.0f;
			Kpp = 0.0f;
			Kpd = 0.1f;
			Khp = 0.0f;
			Khi = 0.0f;
			Kv = 3.0f;
			flip = true;
		}

		/*else if (t < 0.45f) {
			v_ref_r.zero();
			q_ref.from_euler(0.0f,0.0f,psi_ref);
			omega_ref_r.zero();
			a_ref.zero();

			Kap = 0.0f;
			Kad = 0.0f;
			Kpp = 0.0f;
			Kpd = 0.0f;
			Khp = 0.0f;
			Khi = 0.0f;
			Kv = 0.0f;

			flip = true;

		}*/

		else {
			if (flip == true) {
				p_ref_i = p_i;
				v_ref_r.zero();
				start_time_2 = hrt_absolute_time();
				t_1 = Omega_max/Omega_dot_rise;
				t_2 = (4.0f * PI/Omega_max + (Omega_max / Omega_dot_rise - Omega_max / Omega_dot_fall)) / 2.0f;
				t_3 = Omega_max/Omega_dot_fall + t_2;
				flip = false;
			}
			float t_flip = (hrt_absolute_time() - start_time_2) / 1000000.0f;
			if (t_flip < t_1){
				
				Kpp = 0.0f;
				Kpd = 0.0f;
				q_ref.from_euler(Omega_max * powf(t_flip,2.0f) / (2.0f * t_1), 0.0f, psi_ref);
				omega_ref_r = math::Vector<3>(Omega_max * t_flip / t_1, 0.0f, 0.0f);
			}
			else if (t_flip < t_2){
				
				Kpp = 0.0f;
				Kpd = 0.0f;
				q_ref.from_euler(Omega_max * (t_flip - t_1 / 2.0f), 0.0f, psi_ref);
				omega_ref_r = math::Vector<3>(Omega_max, 0.0f, 0.0f);
			}
			else if (t_flip < t_3){
				
				Kpp = 0.0f;
				Kpd = 0.0f;
				q_ref.from_euler(Omega_max * (t_flip - t_1 / 2.0f) - Omega_max * powf(t_flip - t_2, 2.0f) / (2.0f * (t_3 - t_2)), 0.0f, psi_ref);
				omega_ref_r = math::Vector<3>(Omega_max * (1.0f - (t_flip - t_2)/(t_3 - t_2)), 0.0f, 0.0f);
			}
			else{
				q_ref.from_euler(0.0f, 0.0f, psi_ref);
				omega_ref_r.zero();

			}

		}

	}
}


void
MulticopterAttitudeControl::run()
{

	/*
	 * do subscriptions
	 */
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	//_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	//_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	//_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	//_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	//_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);

	if (_gyro_count == 0) {
		_gyro_count = 1;
	}

	for (unsigned s = 0; s < _gyro_count; s++) {
		_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	}

	_sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	_sensor_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

	/* wakeup source: gyro data from sensor selected by the sensor app */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	const hrt_abstime task_start = hrt_absolute_time();
	hrt_abstime last_run = task_start;
	float dt_accumulator = 0.f;
	int loop_counter = 0;

	while (!should_exit()) {

		poll_fds.fd = _sensor_gyro_sub[_selected_gyro];

		/* wait for up to 100ms for data */
		int pret = px4_poll(&poll_fds, 1, 100);

		/* timed out - periodic check for should_exit() */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on gyro changes */
		if (poll_fds.revents & POLLIN) {
			const hrt_abstime now = hrt_absolute_time();
			float dt = (now - last_run) / 1e6f;
			last_run = now;

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy gyro data */
			orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[_selected_gyro], &_sensor_gyro);

			/* check for updates in other topics */
			//parameter_update_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			//vehicle_motor_limits_poll();
			//battery_status_poll();
			vehicle_attitude_poll();
			vehicle_local_position_poll();
			sensor_correction_poll();
			sensor_bias_poll();

			p_i(0) = _v_pos.x;
			p_i(1) = _v_pos.y;
			p_i(2) = _v_pos.z;


			q = _v_att.q;
			Euler = q.to_euler();
			psi = float(Euler(2));

			v_i(0) = _v_pos.vx;
			v_i(1) = _v_pos.vy;
			v_i(2) = _v_pos.vz;

			omega_b(0) = _v_att.rollspeed;
			omega_b(1) = _v_att.pitchspeed;
			omega_b(2) = _v_att.yawspeed;	


		    //_aerobatics_variables2.manuever_type = maneuver_type;
		    //_aerobatics_variables2.timestep = dt;
			

			if (_v_control_mode.flag_control_rates_enabled) {
				//manual_mode();
				//controller(dt);
				if (_vehicle_status.arming_state == 2 && _vehicle_status.rc_signal_lost == false){
					if (int(_manual_control_sp.aux1) == -1) {
						maneuver_type = 0;
						psi_ref = psi;					
					}

					else if (int(_manual_control_sp.aux1) == 1 && int(_manual_control_sp.aux2) == 0) {
                        maneuver_type = man_kind;
                    }

                    else if (int(_manual_control_sp.aux1) == 1 && int(_manual_control_sp.aux2) == 1) {
                        maneuver_type = man_kind2;
                    }

                    else {
                        maneuver_type = 1;
                    }
					controller(dt);
					maneuver_type_old = maneuver_type;	

				}
				else{
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;
				}



				if (_debug_pub != nullptr) {		
					orb_publish(ORB_ID(debug), _debug_pub, &_debug);		
				}		
				else{		
					_debug_pub = orb_advertise(ORB_ID(debug), &_debug);
				}

				if (_aerobatics_variables_pub != nullptr) {		
					orb_publish(ORB_ID(aerobatics_variables), _aerobatics_variables_pub, &_aerobatics_variables);		
				}		
				else{		
					_aerobatics_variables_pub = orb_advertise(ORB_ID(aerobatics_variables), &_aerobatics_variables);
				}
				if (_aerobatics_variables2_pub != nullptr) {		
					orb_publish(ORB_ID(aerobatics_variables2), _aerobatics_variables2_pub, &_aerobatics_variables2);		
				}		
				else{		
					_aerobatics_variables2_pub = orb_advertise(ORB_ID(aerobatics_variables2), &_aerobatics_variables2);
				}

				if (_aerobatics_variables3_pub != nullptr) {		
					orb_publish(ORB_ID(aerobatics_variables3), _aerobatics_variables3_pub, &_aerobatics_variables3);		
				}		
				else{		
					_aerobatics_variables3_pub = orb_advertise(ORB_ID(aerobatics_variables3), &_aerobatics_variables3);
				}



				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}





				}
				/* publish controller status */
				/*rate_ctrl_status_s rate_ctrl_status;
				rate_ctrl_status.timestamp = hrt_absolute_time();
				rate_ctrl_status.rollspeed = _rates_prev(0);
				rate_ctrl_status.pitchspeed = _rates_prev(1);
				rate_ctrl_status.yawspeed = _rates_prev(2);
				rate_ctrl_status.rollspeed_integ = _rates_int(0);
				rate_ctrl_status.pitchspeed_integ = _rates_int(1);
				rate_ctrl_status.yawspeed_integ = _rates_int(2);

				int instance;
				orb_publish_auto(ORB_ID(rate_ctrl_status), &_controller_status_pub, &rate_ctrl_status, &instance, ORB_PRIO_DEFAULT);*/
			}
			#if 0
			if (_v_control_mode.flag_control_termination_enabled) {
				if (!_vehicle_status.is_vtol) {

					_rates_sp.zero();
					_rates_int.zero();
					_thrust_sp = 0.0f;
					_att_control.zero();

					/* publish actuator controls */
					_actuators.control[0] = 0.0f;
					_actuators.control[1] = 0.0f;
					_actuators.control[2] = 0.0f;
					_actuators.control[3] = 0.0f;
					_actuators.timestamp = hrt_absolute_time();
					_actuators.timestamp_sample = _sensor_gyro.timestamp;

					if (!_actuators_0_circuit_breaker_enabled) {
						if (_actuators_0_pub != nullptr) {

							orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
							perf_end(_controller_latency_perf);

						} else if (_actuators_id) {
							_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
						}
					}
				}
			}
			#endif

			/* calculate loop update rate while disarmed or at least a few times (updating the filter is expensive) */
			if (!_v_control_mode.flag_armed || (now - task_start) < 3300000) {
				dt_accumulator += dt;
				++loop_counter;

				if (dt_accumulator > 1.f) {
					const float loop_update_rate = (float)loop_counter / dt_accumulator;
					_loop_update_rate_hz = _loop_update_rate_hz * 0.5f + loop_update_rate * 0.5f;
					dt_accumulator = 0;
					loop_counter = 0;
					_lp_filters_d[0].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_lp_filters_d[1].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
					_lp_filters_d[2].set_cutoff_frequency(_loop_update_rate_hz, _d_term_cutoff_freq.get());
				}
			}

		}

		perf_end(_loop_perf);
	}

	orb_unsubscribe(_v_att_sub);
	//orb_unsubscribe(_v_att_sp_sub);
	orb_unsubscribe(_v_pos_sub);
	//orb_unsubscribe(_v_rates_sp_sub);
	orb_unsubscribe(_v_control_mode_sub);
	//orb_unsubscribe(_params_sub);
	orb_unsubscribe(_manual_control_sp_sub);
	orb_unsubscribe(_vehicle_status_sub);
	//orb_unsubscribe(_motor_limits_sub);
	//orb_unsubscribe(_battery_status_sub);

	for (unsigned s = 0; s < _gyro_count; s++) {
		orb_unsubscribe(_sensor_gyro_sub[s]);
	}

	orb_unsubscribe(_sensor_correction_sub);
	orb_unsubscribe(_sensor_bias_sub);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ATTITUDE_CONTROL,
					   1700,
					   (px4_main_t)&run_trampoline,
					   (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterAttitudeControl *MulticopterAttitudeControl::instantiate(int argc, char *argv[])
{
	return new MulticopterAttitudeControl();
}

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
