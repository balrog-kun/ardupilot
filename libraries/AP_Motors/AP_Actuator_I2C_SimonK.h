// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_ACTUATOR_I2C_SIMONK_H__
#define __AP_ACTUATOR_I2C_SIMONK_H__

#include "AP_Actuator_Channel.h"

class AP_Actuator_I2C_SimonK : public AP_Actuator_Channel {
public:
    AP_Actuator_I2C_SimonK(uint8_t _motor_id, uint8_t pole_count) :
        motor_id(_motor_id), motor_poles(pole_count),
	last_rev_count(0), last_rev_count_ts(0) {}

    /* On a multislave bus you can't just disable a single slave */
    void enable_ch() {}
    void disable_ch() {}

    /* Set the value the actuator is driven with */
    void write(int16_t value);

    /* No output frequency */
    void set_freq(uint16_t) {}

    /*
     * Read the revolution/commutation counter and calculate mean
     * Revolutions Per Sec.  Multiply the return value by 60 to get RPM.
     */
    uint16_t read_rps();
    /* Read the ESC input voltage in Volts times 100 */
    uint16_t read_vbat();

    /* Check actuator is present/alive */
    bool check_alive();

    /* Reverse thrust accepted */
    bool capability_reverse() const { return true; }

protected:
    uint8_t motor_id;
    uint8_t motor_poles;
    uint16_t last_rev_count;
    uint32_t last_rev_count_ts;
};

#endif
