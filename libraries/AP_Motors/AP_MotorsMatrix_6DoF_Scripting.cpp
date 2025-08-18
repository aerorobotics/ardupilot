/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/vectorN.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <stdio.h>
#endif

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsMatrix_6DoF_Scripting::var_info[] = {
    AP_GROUPINFO("CA_TILT_0", 1, AP_MotorsMatrix_6DoF_Scripting, _tilt0,  20.0),
    AP_GROUPINFO("CA_NOZZ_0", 2, AP_MotorsMatrix_6DoF_Scripting, _nozzle0,  0.0),
    AP_GROUPINFO("CA_PIVOT_0", 3, AP_MotorsMatrix_6DoF_Scripting, _fixed_pivot0,  20.0),
    AP_GROUPINFO("CA_T_MAX", 4, AP_MotorsMatrix_6DoF_Scripting, _thrust_max,  550.0),
    AP_GROUPINFO("CA_T_IDLE", 5, AP_MotorsMatrix_6DoF_Scripting, _thrust_idle,  28.0),
    AP_GROUPINFO("CA_MASS", 6, AP_MotorsMatrix_6DoF_Scripting, _mass,  150.0),
    AP_GROUPINFO("CA_SV_TL0_MAXA", 7, AP_MotorsMatrix_6DoF_Scripting, _sv_tl0_max_a,  45.0),
    AP_GROUPINFO("CA_SV_TL1_MAXA", 8, AP_MotorsMatrix_6DoF_Scripting, _sv_tl1_max_a,  0.0),
    AP_GROUPINFO("CA_SV_TL2_MAXA", 9, AP_MotorsMatrix_6DoF_Scripting, _sv_tl2_max_a,  45.0),
    AP_GROUPINFO("CA_SV_TL3_MAXA", 10, AP_MotorsMatrix_6DoF_Scripting, _sv_tl3_max_a,  0.0),
    AP_GROUPINFO("CA_SV_TL4_MAXA", 11, AP_MotorsMatrix_6DoF_Scripting, _sv_tl4_max_a,  20.0),
    AP_GROUPINFO("CA_SV_TL5_MAXA", 12, AP_MotorsMatrix_6DoF_Scripting, _sv_tl5_max_a,  20.0),
    AP_GROUPINFO("CA_SV_TL6_MAXA", 13, AP_MotorsMatrix_6DoF_Scripting, _sv_tl6_max_a,  20.0),
    AP_GROUPINFO("CA_SV_TL7_MAXA", 14, AP_MotorsMatrix_6DoF_Scripting, _sv_tl7_max_a,  20.0),
    AP_GROUPINFO("CA_SV_TL0_MINA", 15, AP_MotorsMatrix_6DoF_Scripting, _sv_tl0_min_a,  0.0),
    AP_GROUPINFO("CA_SV_TL1_MINA", 16, AP_MotorsMatrix_6DoF_Scripting, _sv_tl1_min_a,  -45.0),
    AP_GROUPINFO("CA_SV_TL2_MINA", 17, AP_MotorsMatrix_6DoF_Scripting, _sv_tl2_min_a,  0.0),
    AP_GROUPINFO("CA_SV_TL3_MINA", 18, AP_MotorsMatrix_6DoF_Scripting, _sv_tl3_min_a,  -45.0),
    AP_GROUPINFO("CA_SV_TL4_MINA", 19, AP_MotorsMatrix_6DoF_Scripting, _sv_tl4_min_a,  -20.0),
    AP_GROUPINFO("CA_SV_TL5_MINA", 20, AP_MotorsMatrix_6DoF_Scripting, _sv_tl5_min_a,  -20.0),
    AP_GROUPINFO("CA_SV_TL6_MINA", 21, AP_MotorsMatrix_6DoF_Scripting, _sv_tl6_min_a,  -20.0),
    AP_GROUPINFO("CA_SV_TL7_MINA", 22, AP_MotorsMatrix_6DoF_Scripting, _sv_tl7_min_a,  -20.0),
    AP_GROUPINFO("CA_FX_SCALE", 23, AP_MotorsMatrix_6DoF_Scripting, _fx_scale,  1.0),
    AP_GROUPINFO("CA_FY_SCALE", 24, AP_MotorsMatrix_6DoF_Scripting, _fy_scale,  1.0),
    AP_GROUPINFO("CA_FZ_SCALE", 25, AP_MotorsMatrix_6DoF_Scripting, _fz_scale,  1.0),
    AP_GROUPINFO("CA_MX_SCALE", 26, AP_MotorsMatrix_6DoF_Scripting, _mx_scale,  8000.0),
    AP_GROUPINFO("CA_MY_SCALE", 27, AP_MotorsMatrix_6DoF_Scripting, _my_scale,  8000.0),
    AP_GROUPINFO("CA_MZ_SCALE", 28, AP_MotorsMatrix_6DoF_Scripting, _mz_scale,  8000.0),
    AP_GROUPEND
};

void AP_MotorsMatrix_6DoF_Scripting::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: // disarmed
        case SpoolState::GROUND_IDLE: // armed, but on ground       
        {
            // no output, cant spin up for ground idle because we don't know which way motors should be spining
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (_reversible[i]) {
                        // revesible motor can provide both positive and negative thrust, +- spin max, spin min does not apply
                        if (is_positive(_thrust_rpyt_out[i])) { 
                            _actuator[i] = thr_lin.apply_thrust_curve_and_volt_scaling(_thrust_rpyt_out[i]) * thr_lin.get_spin_max();

                        } else if (is_negative(_thrust_rpyt_out[i])) {
                            _actuator[i] = -thr_lin.apply_thrust_curve_and_volt_scaling(-_thrust_rpyt_out[i]) * thr_lin.get_spin_max();

                        } else {
                            _actuator[i] = 0.0f;
                        }
                    } else {
                        // motor can only provide trust in a single direction, spin min to spin max as 'normal' copter
                         _actuator[i] = thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]);
                    }
                }
            }
            break;
    }

    printf("spool state %d \n", static_cast<uint8_t>(_spool_state));

    // Send to each motor
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _actuator[i] * 4500);
        }
    }

}
// output_armed - sends commands to the motors
void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   right_thrust;               // right thrust input value, +/- 1.0

    const float compensation_gain = thr_lin.get_compensation_gain(); // compensation for battery voltage and altitude
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;
    forward_thrust = get_forward() * throttle_thrust;
    right_thrust = get_lateral() * throttle_thrust;

    Matrix3f rot;
    Vector3f thrust_vec;
    rot.from_euler312(_roll_offset, _pitch_offset, 0.0f);
    thrust_vec.x = forward_thrust;
    thrust_vec.y = right_thrust;
    thrust_vec.z = throttle_thrust;
    thrust_vec.z *= -1.0f;
    thrust_vec = rot * thrust_vec;

    Vector3f control_sp_thrust = thrust_vec;

    Vector3f acc_body;
    const float _hover_thrust = 0.7;
	static constexpr float CONSTANTS_ONE_G = 9.81f; // Standard gravity in m/s^2
    if (fabsf(_hover_thrust) > 1e-6f) { // Avoid division by zero
		acc_body = (control_sp_thrust + Vector3f(0.f, 0.f, _hover_thrust)) * CONSTANTS_ONE_G / _hover_thrust;

	} else {
		// Handle the case where hover_thrust is zero or very small, maybe set acc_body to zero or some default
		acc_body.zero();
	}

    // TODO: fill _R_bn with the correct rotation matrix
	Matrix3f _R_bn; // Assuming default constructor initializes appropriately (e.g., identity)
    _R_bn.identity();

	Vector3f acc_ned = _R_bn.transposed() * acc_body + Vector3f(0.f, 0.f, -CONSTANTS_ONE_G);
	Vector3f force_ned = acc_ned * _mass;
    VectorN<float, n_outputs_> control_sp_ned; // 6dof, so 6 control setpoints 
    VectorN<float, n_outputs_> control_trim; 

    

	control_sp_ned[0] = roll_thrust;
	control_sp_ned[1] = pitch_thrust;
	control_sp_ned[2] = yaw_thrust;
	control_sp_ned[3] = force_ned[0];
	control_sp_ned[4] = force_ned[1];
	control_sp_ned[5] = force_ned[2];

    control_trim.zero();
    control_trim[5] = -CONSTANTS_ONE_G * _mass;

    // Compute the actuator trim in the actuator space

    // Vector<float, NUM_ACTUATORS> _actuator_trim_unnormalized = _actuator_trim;
    VectorN<float, n_servos_> _actuator_trim_unnormalized;
    for (int i=0; i<4; i++){
        _actuator_trim_unnormalized[i] = nominal_tilt_[i]; 
        _actuator_trim_unnormalized[4+i] = nominal_nozzle_[i]; 
    }

	VectorN<float, n_outputs_> control_needed_after_trim = control_sp_ned - control_trim;

    // scale the control needed to the actuator space  [ N and N.m ]
	control_needed_after_trim[0] = control_needed_after_trim[0]*_mx_scale;
	control_needed_after_trim[1] = control_needed_after_trim[1]*_my_scale;
	control_needed_after_trim[2] = control_needed_after_trim[2]*_mz_scale;
	control_needed_after_trim[3] = control_needed_after_trim[3]*_fx_scale;
	control_needed_after_trim[4] = control_needed_after_trim[4]*_fy_scale;
	control_needed_after_trim[5] = control_needed_after_trim[5]*_fz_scale;

    

	// Allocate. Using raw 2D array for _mix because Ardupilot doesn't support non-square matrix
    // This block is equivalent to _actuator_sp = _actuator_trim_unnormalized + mix @ control_needed_after trim
    VectorN<float, n_servos_> _actuator_sp;
    for (int i = 0; i < n_servos_; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < n_outputs_; ++j) {
            sum += _mix[i][j] * control_needed_after_trim[j];
        }
        _actuator_sp[i] = _actuator_trim_unnormalized[i] + sum;
    }

	// clip to [min, max]
	for (int i=0; i<4; i++){
		if (_actuator_sp[i] < tilt_min_[i]){
			_actuator_sp[i] = tilt_min_[i];
		}
		if (_actuator_sp[i] > tilt_max_[i]){
			_actuator_sp[i] = tilt_max_[i];
		}
		if (_actuator_sp[4+i] < nozzle_min_[i]){
			_actuator_sp[4+i] = nozzle_min_[i];
		}
		if (_actuator_sp[4+i] > nozzle_max_[i]){
			_actuator_sp[4+i] = nozzle_max_[i];
		}
	}

    // Normalize actuator setpoint back to the range [-1, 1]
	for (int i=0; i<4; i++){
         if (fabsf(tilt_max_[i] - tilt_min_[i]) > 1e-6f) { // Avoid division by zero
    		_actuator_sp[i] = 2 * (_actuator_sp[i] - tilt_min_[i]) / (tilt_max_[i] - tilt_min_[i]) - 1.0f;
         }
         else{
            _actuator_sp[i] = 0.0f;
         }
         if (fabsf(nozzle_max_[i] - nozzle_min_[i]) > 1e-6f) { // Avoid division by zero
    		_actuator_sp[4+i] = 2 * (_actuator_sp[4+i] - nozzle_min_[i]) / (nozzle_max_[i] - nozzle_min_[i]) - 1.0f;
         }
         else{
            _actuator_sp[4+i] = 0.0f;
         }
	}

    for (int i = 0; i < 4; i++) {
        if (motor_enabled[i]) {
            if (fabsf(_thrust_max - _thrust_idle) > 1e-6f) { // Avoid division by zero
                _thrust_rpyt_out[i] = (0.282 * _mass * CONSTANTS_ONE_G - _thrust_idle) / (_thrust_max - _thrust_idle); // thrusters
            }
            _thrust_rpyt_out[4+i] = _actuator_sp[i]; // tilt
            _thrust_rpyt_out[4+4+i] = _actuator_sp[4+i]; // nozzle
        }
    }

    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // printf("------------------------------------------------------\n");
        // printf("throttle thrust \n\t%.3f\n", static_cast<double>(throttle_thrust));

        printf("thruster setpoint \n\t%.3f, %.3f, %.3f, %.3f\n",
            static_cast<double>(_actuator[0]),
            static_cast<double>(_actuator[1]),
            static_cast<double>(_actuator[2]),
            static_cast<double>(_actuator[3]));

        printf("control_sp_thrust \n\t%.3f, %.3f, %.3f\n",
            static_cast<double>(control_sp_thrust[0]),
            static_cast<double>(control_sp_thrust[1]),
            static_cast<double>(control_sp_thrust[2]));

        // printf("acc_ned \n\t%.3f, %.3f, %.3f\n",
        //     static_cast<double>(acc_ned[0]),
        //     static_cast<double>(acc_ned[1]),
        //     static_cast<double>(acc_ned[2]));

        // printf("control_sp_ned \n\t%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
        //     static_cast<double>(control_sp_ned[0]),
        //     static_cast<double>(control_sp_ned[1]),
        //     static_cast<double>(control_sp_ned[2]),
        //     static_cast<double>(control_sp_ned[3]),
        //     static_cast<double>(control_sp_ned[4]),
        //     static_cast<double>(control_sp_ned[5]));
        
        // printf("control_trim \n\t%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
        //     static_cast<double>(control_trim[0]),
        //     static_cast<double>(control_trim[1]),
        //     static_cast<double>(control_trim[2]),
        //     static_cast<double>(control_trim[3]),
        //     static_cast<double>(control_trim[4]),
        //     static_cast<double>(control_trim[5]));

        // printf("control needed after trim \n\t%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
        //     static_cast<double>(control_needed_after_trim[0]),
        //     static_cast<double>(control_needed_after_trim[1]),
        //     static_cast<double>(control_needed_after_trim[2]),
        //     static_cast<double>(control_needed_after_trim[3]),
        //     static_cast<double>(control_needed_after_trim[4]),
        //     static_cast<double>(control_needed_after_trim[5]));

        // printf("actuator_sp before clipping (tilt, nozzle) \n\t%.3f, %.3f\n\t%.3f, %.3f\n\t%.3f, %.3f\n\t%.3f, %.3f \n",
        //     static_cast<double>(_actuator_sp[0]),
        //     static_cast<double>(_actuator_sp[1]),
        //     static_cast<double>(_actuator_sp[2]),
        //     static_cast<double>(_actuator_sp[3]),
        //     static_cast<double>(_actuator_sp[4]),
        //     static_cast<double>(_actuator_sp[5]),
        //     static_cast<double>(_actuator_sp[6]),
        //     static_cast<double>(_actuator_sp[7]));

        // printf("force_ned \n\t%.3f, %.3f, %.3f\n",
        //     static_cast<double>(force_ned[0]),
        //     static_cast<double>(force_ned[1]),
        //     static_cast<double>(force_ned[2]));


        // printf("actuator_trim rad (tilt, nozzle) \n\t%.3f, %.3f\n\t%.3f, %.3f\n\t%.3f, %.3f\n\t%.3f, %.3f \n",
        //     static_cast<double>(_actuator_trim_unnormalized[0]),
        //     static_cast<double>(_actuator_trim_unnormalized[4]),
        //     static_cast<double>(_actuator_trim_unnormalized[1]),
        //     static_cast<double>(_actuator_trim_unnormalized[5]),
        //     static_cast<double>(_actuator_trim_unnormalized[2]),
        //     static_cast<double>(_actuator_trim_unnormalized[6]),
        //     static_cast<double>(_actuator_trim_unnormalized[3]),
        //     static_cast<double>(_actuator_trim_unnormalized[7]));

        // printf("torques needed \n\t%.3f, %.3f, %.3f,\nforces needed: \n\t%.3f, %.3f, %.3f\n",
        //     static_cast<double>(control_needed_after_trim[0]),
        //     static_cast<double>(control_needed_after_trim[1]),
        //     static_cast<double>(control_needed_after_trim[2]),
        //     static_cast<double>(control_needed_after_trim[3]),
        //     static_cast<double>(control_needed_after_trim[4]),
        //     static_cast<double>(control_needed_after_trim[5]));
    #endif

}

// sets the roll and pitch offset, this rotates the thrust vector in body frame
// these are typically set such that the throttle thrust vector is earth frame up
void AP_MotorsMatrix_6DoF_Scripting::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

// add_motor, take roll, pitch, yaw, throttle(up), forward, right factors along with a bool if the motor is reversible and the testing order, called from scripting
void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order)
{
    if (initialised_ok()) {
        // don't allow matrix to be changed after init
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        motor_enabled[motor_num] = true;

        _roll_factor[motor_num] = roll_factor;
        _pitch_factor[motor_num] = pitch_factor;
        _yaw_factor[motor_num] = yaw_factor;

        _throttle_factor[motor_num] = throttle_factor;
        _forward_factor[motor_num] = forward_factor;
        _right_factor[motor_num] = right_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // ensure valid motor number is provided
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);

        uint8_t chan;
        if (!SRV_Channels::find_channel(function, chan)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
            return;
        }

        _reversible[motor_num] = reversible;
        if (_reversible[motor_num]) {
            // reversible, set to angle type hard code trim to 1500
            SRV_Channels::set_angle(function, 4500);
            SRV_Channels::set_trim_to_pwm_for(function, 1500);
        } else {
            SRV_Channels::set_range(function, 4500);
        }
        SRV_Channels::set_output_min_max(function, get_pwm_output_min(), get_pwm_output_max());
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::init(uint8_t expected_num_motors) {
    AP_Param::setup_object_defaults(this, var_info);
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    // EVADE Initialization
    nominal_tilt_[0] = DEG2RAD*_tilt0;
	nominal_tilt_[1] = -DEG2RAD*_tilt0;
	nominal_tilt_[2] = DEG2RAD*_tilt0;
	nominal_tilt_[3] = -DEG2RAD*_tilt0;

	nominal_nozzle_[0] = -DEG2RAD*_nozzle0;
	nominal_nozzle_[1] = -DEG2RAD*_nozzle0;
	nominal_nozzle_[2] = DEG2RAD*_nozzle0;
	nominal_nozzle_[3] = DEG2RAD*_nozzle0;
	fixed_pivot_[0] = -DEG2RAD*_fixed_pivot0;
	fixed_pivot_[1] = -DEG2RAD*_fixed_pivot0;
	fixed_pivot_[2] = DEG2RAD*_fixed_pivot0;
	fixed_pivot_[3] = DEG2RAD*_fixed_pivot0;

	tilt_min_[0] = DEG2RAD*_sv_tl0_min_a;
	tilt_min_[1] = DEG2RAD*_sv_tl1_min_a;
	tilt_min_[2] = DEG2RAD*_sv_tl2_min_a;
	tilt_min_[3] = DEG2RAD*_sv_tl3_min_a;
	tilt_max_[0] = DEG2RAD*_sv_tl0_max_a;
	tilt_max_[1] = DEG2RAD*_sv_tl1_max_a;
	tilt_max_[2] = DEG2RAD*_sv_tl2_max_a;
	tilt_max_[3] = DEG2RAD*_sv_tl3_max_a;

	nozzle_min_[0] = DEG2RAD*_sv_tl4_min_a;
	nozzle_min_[1] = DEG2RAD*_sv_tl5_min_a;
	nozzle_min_[2] = DEG2RAD*_sv_tl6_min_a;
	nozzle_min_[3] = DEG2RAD*_sv_tl7_min_a;
	nozzle_max_[0] = DEG2RAD*_sv_tl4_max_a;
	nozzle_max_[1] = DEG2RAD*_sv_tl5_max_a;
	nozzle_max_[2] = DEG2RAD*_sv_tl6_max_a;
	nozzle_max_[3] = DEG2RAD*_sv_tl7_max_a;

    return true;
}

// singleton instance
AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::_singleton;

#endif // AP_SCRIPTING_ENABLED
