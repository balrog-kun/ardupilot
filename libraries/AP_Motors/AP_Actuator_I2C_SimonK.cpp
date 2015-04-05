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

#include <AP_HAL.h>
#include <AP_Math.h>
#include "AP_Actuator_I2C_SimonK.h"

extern const AP_HAL::HAL &hal;

/*
 * The following values are needed to correctly calculate ESC's input
 * voltage and temperature.  The defaults below match the AfroESC series
 * with a 10k Ohm NTC, you'll need to adjust these values for your ESC
 * model if you want to use the voltage or temperature feedback.
 */
#define V_REF       5.0
#define OHM_TO_GND  3300
#define OHM_TO_VBAT 18000
#define NTC_B       4050
#define NTC_R_INF   0.0126f /* 10000 Ohm * e ^ (-4050 K / (25 C + 273.15 K)) */

/* I2C register map */
#define ESC_REG_THROTTLE  0x00 /* Write-only */
#define ESC_REG_REV_COUNT 0x02 /* Read-only */
#define ESC_REG_VBAT      0x04 /* Read-only */
#define ESC_REG_TEMP      0x06 /* Read-only */
#define ESC_REG_ID        0x08 /* Read-only */

void AP_Actuator_I2C_SimonK::write(int16_t value) {
    /* Scale from the 1000-2000 input range to 0-32K output */
    uint16_t out_value = (max(value, 1000) - 1000) << 5;
    /* Write the MSB and the LSB of new value to the throttle register */
    uint8_t regs[] = { (uint8_t) (out_value >> 8), (uint8_t) out_value };
    if (hal.i2c->writeRegisters(motor_id, ESC_REG_THROTTLE, 2, regs))
        return; /* Write error */
}

uint16_t AP_Actuator_I2C_SimonK::read_rps() {
    uint16_t rev_count, diff;
    uint8_t buf[2];
    uint32_t now, timediff;

    if (hal.i2c->readRegister(motor_id, ESC_REG_REV_COUNT, buf))
        return 0;

    now = hal.scheduler->micros();
    timediff = now - last_rev_count_ts;
    last_rev_count_ts = now;

    rev_count = ((uint16_t) buf[0] << 8) | buf[1];
    diff = rev_count - last_rev_count;
    last_rev_count = rev_count;

    return (uint32_t) diff * (1000000 * 2) / ((timediff * motor_poles) ?: 1);
}

uint16_t AP_Actuator_I2C_SimonK::read_vbat() {
    uint16_t raw_adc;
    uint8_t buf[2];

    if (hal.i2c->readRegister(motor_id, ESC_REG_VBAT, buf))
        return 0;

    raw_adc = ((uint16_t) buf[0] << 8) | buf[1];
    return ((uint32_t) raw_adc * (uint16_t) (V_REF * 100) * 100) /
        ((0x10000LL * 100 * OHM_TO_GND) / (OHM_TO_GND + OHM_TO_VBAT));
}

bool AP_Actuator_I2C_SimonK::check_alive() {
    uint8_t id;

    /* Read the ID register, check if value is 0xab */
    if (hal.i2c->readRegister(motor_id, ESC_REG_ID, &id))
        return false;

    return id == 0xab;
}
