// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *       AP_MotorsSingle.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_MotorsSingle.h"

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_MotorsSingle::var_info[] PROGMEM = {
    // 0 was used by TB_RATIO
    // 1,2,3 were used by throttle curve

    // @Param: SPIN_ARMED
    // @DisplayName: Motors always spin when armed
    // @Description: Controls whether motors always spin when armed (must be below THR_MIN)
    // @Values: 0:Do Not Spin,70:VerySlow,100:Slow,130:Medium,150:Fast
    // @User: Standard
    AP_GROUPINFO("SPIN_ARMED", 5, AP_MotorsSingle, _spin_when_armed, AP_MOTORS_SPIN_WHEN_ARMED),

    // @Param: REV_ROLL
    // @DisplayName: Reverse roll feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_ROLL", 6, AP_MotorsSingle, _rev_roll, AP_MOTORS_SING_POSITIVE),

    // @Param: REV_PITCH
    // @DisplayName: Reverse pitch feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_PITCH", 7, AP_MotorsSingle, _rev_pitch, AP_MOTORS_SING_POSITIVE),

	// @Param: REV_YAW
    // @DisplayName: Reverse yaw feedback 
    // @Description: Ensure the feedback is negative
    // @Values: -1:Opposite direction,1:Same direction
    AP_GROUPINFO("REV_YAW", 8, AP_MotorsSingle, _rev_yaw, AP_MOTORS_SING_POSITIVE),

	// @Param: SV_SPEED
    // @DisplayName: Servo speed 
    // @Description: Servo update speed in hz
    // @Values: 50, 125, 250
    AP_GROUPINFO("SV_SPEED", 9, AP_MotorsSingle, _servo_speed, AP_MOTORS_SINGLE_SPEED_DIGITAL_SERVOS),

    AP_GROUPEND
};
// init
void AP_MotorsSingle::Init()
{
    // call parent Init function to set-up throttle curve
    AP_Motors::Init();

    // set update rate for the 3 motors (but not the servo on channel 7)
    set_update_rate(_speed_hz);

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_7] = true;

    // we set four servos to angle
    _servo1.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo2.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo3.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo4.set_type(RC_CHANNEL_TYPE_ANGLE);
    _servo1.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo2.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo3.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);
    _servo4.set_angle(AP_MOTORS_SINGLE_SERVO_INPUT_RANGE);

    // disable CH7 from being used as an aux output (i.e. for camera gimbal, etc)
    RC_Channel_aux::disable_aux_channel(CH_7);
}

// set update rate to motors - a value in hertz
void AP_MotorsSingle::set_update_rate( uint16_t speed_hz )
{
    // record requested speed
    _speed_hz = speed_hz;

    // set update rate for the 3 motors (but not the servo on channel 7)
    _rc_out[AP_MOTORS_MOT_1]->set_freq(_servo_speed);
    _rc_out[AP_MOTORS_MOT_2]->set_freq(_servo_speed);
    _rc_out[AP_MOTORS_MOT_3]->set_freq(_servo_speed);
    _rc_out[AP_MOTORS_MOT_4]->set_freq(_servo_speed);

    _rc_out[AP_MOTORS_MOT_7]->set_freq(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void AP_MotorsSingle::enable()
{
    // enable output channels
    _rc_out[AP_MOTORS_MOT_1]->enable_ch();
    _rc_out[AP_MOTORS_MOT_2]->enable_ch();
    _rc_out[AP_MOTORS_MOT_3]->enable_ch();
    _rc_out[AP_MOTORS_MOT_4]->enable_ch();
    _rc_out[AP_MOTORS_MOT_7]->enable_ch();
}

// output_min - sends minimum values out to the motor and trim values to the servos
void AP_MotorsSingle::output_min()
{
    // send minimum value to each motor
    _rc_out[AP_MOTORS_MOT_1]->write(_servo1.radio_trim);
    _rc_out[AP_MOTORS_MOT_2]->write(_servo2.radio_trim);
    _rc_out[AP_MOTORS_MOT_3]->write(_servo3.radio_trim);
    _rc_out[AP_MOTORS_MOT_4]->write(_servo4.radio_trim);
    _rc_out[AP_MOTORS_MOT_7]->write(_rc_throttle.radio_min);
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t AP_MotorsSingle::get_motor_mask()
{
    // single copter uses channels 1,2,3,4 and 7
    return (1U << 0 | 1U << 1 | 1U << 2 | 1U << 3 | 1U << 6);
}

// output_armed - sends commands to the motors
void AP_MotorsSingle::output_armed()
{
    int16_t out_min = _rc_throttle.radio_min + _min_throttle;
    int16_t motor_out;  // main motor output

    // Throttle is 0 to 1000 only
    if (_rc_throttle.servo_out <= 0) {
            _rc_throttle.servo_out = 0;
            limit.throttle_lower = true;
        }
    if (_rc_throttle.servo_out >= _max_throttle) {
        _rc_throttle.servo_out = _max_throttle;
        limit.throttle_upper = true;
    }

    // capture desired throttle from receiver
    _rc_throttle.calc_pwm();

    // if we are not sending a throttle output, we cut the motors
    if(_rc_throttle.servo_out == 0) {
        // range check spin_when_armed
        if (_spin_when_armed < 0) {
            _spin_when_armed = 0;
        }
        if (_spin_when_armed > _min_throttle) {
            _spin_when_armed = _min_throttle;
        }

        // throttle is limited
        motor_out = _rc_throttle.radio_min + _spin_when_armed;
    }else{
        // check if throttle is at or below limit
        if (_rc_throttle.servo_out <= _min_throttle) {
            limit.throttle_lower = true;
            _rc_throttle.servo_out = _min_throttle;
            _rc_throttle.calc_pwm();    // recalculate radio.out
        }

		//motor
        motor_out = _rc_throttle.radio_out;

        // adjust for thrust curve and voltage scaling
        motor_out = apply_thrust_curve_and_volt_scaling(motor_out, out_min, _rc_throttle.radio_max);

        // ensure motor doesn't drop below a minimum value and stop
        motor_out = max(motor_out, out_min);
    }

    // front servo
    _servo1.servo_out = _rev_roll*_rc_roll.servo_out + _rev_yaw*_rc_yaw.servo_out;
    // right servo
    _servo2.servo_out = _rev_pitch*_rc_pitch.servo_out + _rev_yaw*_rc_yaw.servo_out;
    // rear servo
    _servo3.servo_out = -_rev_roll*_rc_roll.servo_out + _rev_yaw*_rc_yaw.servo_out;
    // left servo
    _servo4.servo_out = -_rev_pitch*_rc_pitch.servo_out + _rev_yaw*_rc_yaw.servo_out;

    _servo1.calc_pwm();
    _servo2.calc_pwm();
    _servo3.calc_pwm();
    _servo4.calc_pwm();

    // send output to each actuator
    _rc_out[AP_MOTORS_MOT_1]->write(_servo1.radio_out);
    _rc_out[AP_MOTORS_MOT_2]->write(_servo2.radio_out);
    _rc_out[AP_MOTORS_MOT_3]->write(_servo3.radio_out);
    _rc_out[AP_MOTORS_MOT_4]->write(_servo4.radio_out);
    _rc_out[AP_MOTORS_MOT_7]->write(motor_out);
}

// output_disarmed - sends commands to the motors
void AP_MotorsSingle::output_disarmed()
{
    // Send minimum values to all motors
    output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsSingle::output_test(uint8_t motor_seq, int16_t pwm)
{
    // exit immediately if not armed
    if (!_flags.armed) {
        return;
    }

    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            _rc_out[AP_MOTORS_MOT_1]->write(pwm);
            break;
        case 2:
            // flap servo 2
            _rc_out[AP_MOTORS_MOT_2]->write(pwm);
            break;
        case 3:
            // flap servo 3
            _rc_out[AP_MOTORS_MOT_3]->write(pwm);
            break;
        case 4:
            // flap servo 4
            _rc_out[AP_MOTORS_MOT_4]->write(pwm);
            break;
        case 5:
            // spin main motor
            _rc_out[AP_MOTORS_MOT_7]->write(pwm);
            break;
        default:
            // do nothing
            break;
    }
}
