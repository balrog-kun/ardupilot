// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_MotorsY6.h
/// @brief	Motor control class for Y6 frames

#ifndef __AP_MOTORS_Y6_H__
#define __AP_MOTORS_Y6_H__

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_MotorsMatrix.h>    // Parent Motors Matrix library

#define AP_MOTORS_Y6_YAW_DIRECTION 1    // this really should be a user selectable parameter

/// @class      AP_MotorsY6
class AP_MotorsY6 : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsY6(RC_Channel& rc_roll, RC_Channel& rc_pitch, RC_Channel& rc_throttle, RC_Channel& rc_yaw, AP_Actuator_Channel* rc_out[], uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) : AP_MotorsMatrix(rc_roll, rc_pitch, rc_throttle, rc_yaw, rc_out, loop_rate, speed_hz) {
    };

    // setup_motors - configures the motors for a Y6
    virtual void        setup_motors();

protected:

};

#endif  // AP_MOTORSY6
