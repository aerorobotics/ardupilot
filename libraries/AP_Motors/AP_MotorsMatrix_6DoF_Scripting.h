#pragma once
#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_MotorsMatrix.h"

class AP_MotorsMatrix_6DoF_Scripting : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsMatrix_6DoF_Scripting(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix 6DoF must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_6DoF_Scripting *get_singleton() {
        return _singleton;
    }

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // sets the roll and pitch offset, this rotates the thrust vector in body frame
    // these are typically set such that the throttle thrust vector is earth frame up
    void set_roll_pitch(float roll_deg, float pitch_deg) override;

    // add_motor using raw roll, pitch, throttle and yaw factors, to be called from scripting
    void add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order);

    // if the expected number of motors have been setup then set as initalized
    bool init(uint8_t expected_num_motors) override;

    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // output - sends commands to the motors
    void output_armed_stabilizing() override;

    // nothing to do for setup, scripting will mark as initalized when done
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    const char* _get_frame_string() const override { return "6DoF scripting"; }

    float _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];      // each motors contribution to forward thrust
    float _right_factor[AP_MOTORS_MAX_NUM_MOTORS];        // each motors contribution to right thrust

    // true if motor is revesible, it can go from -Spin max to +Spin max, if false motor is can go from Spin min to Spin max
    bool _reversible[AP_MOTORS_MAX_NUM_MOTORS];

    // store last values to allow deadzone
    float _last_thrust_out[AP_MOTORS_MAX_NUM_MOTORS];

    // Current offset angles, radians
    float _roll_offset;
    float _pitch_offset;

    // EVADE
    static constexpr int n_thrusters_ = 4;
    static constexpr int n_servos_ = 8;
	static constexpr int n_outputs_ = 6;
    static constexpr float DEG2RAD = 3.14159265f / 180.0f;

    AP_Float _tilt0;
	AP_Float _nozzle0;
	AP_Float _fixed_pivot0;

    AP_Float _thrust_max;
    AP_Float _thrust_idle;
    AP_Float _mass;

    AP_Float _fx_scale;
    AP_Float _fy_scale;
    AP_Float _fz_scale;
    AP_Float _mx_scale;
    AP_Float _my_scale;
    AP_Float _mz_scale;

    AP_Float _sv_tl0_max_a;
    AP_Float _sv_tl1_max_a;
    AP_Float _sv_tl2_max_a;
    AP_Float _sv_tl3_max_a;
    AP_Float _sv_tl4_max_a;
    AP_Float _sv_tl5_max_a;
    AP_Float _sv_tl6_max_a;
    AP_Float _sv_tl7_max_a;
    AP_Float _sv_tl0_min_a;
    AP_Float _sv_tl1_min_a;
    AP_Float _sv_tl2_min_a;
    AP_Float _sv_tl3_min_a;
    AP_Float _sv_tl4_min_a;
    AP_Float _sv_tl5_min_a;
    AP_Float _sv_tl6_min_a;
    AP_Float _sv_tl7_min_a;


    float nominal_tilt_[n_thrusters_];
	float nominal_nozzle_[n_thrusters_];
	float fixed_pivot_[n_thrusters_];
	float tilt_min_[n_thrusters_];
	float tilt_max_[n_thrusters_];
	float nozzle_min_[n_thrusters_];
	float nozzle_max_[n_thrusters_];
	float nominal_thrust_[n_thrusters_];

    // Allocation matrix (mixer) 
    float _mix[n_servos_][n_outputs_] = {
        {0.000840    ,    0.000000    ,    0.000198    ,    -0.000581   ,    0.000623   ,   0.000796 },
        {-0.000840   ,    0.000000    ,    0.000198    ,    -0.000581   ,    -0.000623  ,   -0.000796},
        {-0.000840   ,    -0.000000   ,    -0.000198   ,    -0.000581   ,    -0.000623  ,   0.000796 },
        {0.000840    ,    -0.000000   ,    -0.000198   ,    -0.000581   ,    0.000623   ,   -0.000796},
        {-0.000105   ,    0.001152    ,    0.000140    ,    -0.000652   ,    0.000503   ,   -0.000847},
        {-0.000105   ,    -0.001152   ,    -0.000140   ,    0.000652    ,    0.000503   ,   -0.000847},
        {-0.000105   ,    -0.001152   ,    0.000140    ,    0.000652    ,    0.000503   ,   0.000847 },
        {-0.000105   ,    0.001152    ,    -0.000140   ,    -0.000652   ,    0.000503   ,   0.000847 }
    };

private:
    static AP_MotorsMatrix_6DoF_Scripting *_singleton;

};

#endif // AP_SCRIPTING_ENABLED
