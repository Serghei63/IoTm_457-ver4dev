/*

BL0937

Copyright (C) 2016-2018 by Xose Pérez <xose dot perez at gmail dot com>

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

#include <Arduino.h>
#include "BL0937lib.h"

void BL0937::begin(
    unsigned char cf_pin,
    unsigned char cf1_pin,
    unsigned char sel_pin,
    unsigned char currentWhen,
    bool use_interrupts,
    unsigned long pulse_timeout
    ) {

    _cf_pin = cf_pin;
    _cf1_pin = cf1_pin;
    _sel_pin = sel_pin;
    _current_mode = currentWhen;
    _use_interrupts = use_interrupts;
    _pulse_timeout = pulse_timeout;

    pinMode(_cf_pin, INPUT_PULLUP);
    pinMode(_cf1_pin, INPUT_PULLUP);
    pinMode(_sel_pin, OUTPUT);

    _calculateDefaultMultipliers();

    _mode = _current_mode;
    digitalWrite(_sel_pin, _mode);


}

void BL0937::setMode(bl0937_mode_t mode) {
    _mode = (mode == MODE_CURRENT) ? _current_mode : 1 - _current_mode;
    digitalWrite(_sel_pin, _mode);
    if (_use_interrupts) {
        _last_cf1_interrupt = _first_cf1_interrupt = micros();
    }
}

bl0937_mode_t BL0937::getMode() {
    return (_mode == _current_mode) ? MODE_CURRENT : MODE_VOLTAGE;
}

bl0937_mode_t BL0937::toggleMode() {
    bl0937_mode_t new_mode = getMode() == MODE_CURRENT ? MODE_VOLTAGE : MODE_CURRENT;
    setMode(new_mode);
    return new_mode;
}

double BL0937::getCurrent() {

    // Power measurements are more sensitive to switch offs,
    // so we first check if power is 0 to set _current to 0 too
    if (_power == 0) {
        _current_pulse_width = 0;
        getActivePower();

    } else if (_use_interrupts) {
        _checkCF1Signal();

    } else if (_mode == _current_mode) {
        _current_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    }

    _current = (_current_pulse_width > 0) ? _current_multiplier / _current_pulse_width : 0;
    return _current;

}

unsigned int BL0937::getVoltage() {
    if (_use_interrupts) {
        _checkCF1Signal();
    } else if (_mode != _current_mode) {
        _voltage_pulse_width = pulseIn(_cf1_pin, HIGH, _pulse_timeout);
    }
    _voltage = (_voltage_pulse_width > 0) ? _voltage_multiplier / _voltage_pulse_width : 0;
    return _voltage;
}

unsigned int BL0937::getActivePower() {
    if (_use_interrupts) {
        _checkCFSignal();
    } else {
        _power_pulse_width = pulseIn(_cf_pin, HIGH, _pulse_timeout);
    }
    _power = (_power_pulse_width > 0) ? _power_multiplier / _power_pulse_width : 0;
    return _power;
}

unsigned int BL0937::getApparentPower() {
    double current = getCurrent();
    unsigned int voltage = getVoltage();
    return voltage * current;
}

unsigned int BL0937::getReactivePower() {
    unsigned int active = getActivePower();
    unsigned int apparent = getApparentPower();
    if (apparent > active) {
        return sqrt(apparent * apparent - active * active);
    } else {
        return 0;
    }
}

double BL0937::getPowerFactor() {
    unsigned int active = getActivePower();
    unsigned int apparent = getApparentPower();
    if (active > apparent) return 1;
    if (apparent == 0) return 0;
    return (double) active / apparent;
}

unsigned long BL0937::getEnergy() {

    // Counting pulses only works in IRQ mode
    if (!_use_interrupts) return 0;

    /*
    Pulse count is directly proportional to energy:
    P = m*f (m=power multiplier, f = Frequency)
    f = N/t (N=pulse count, t = time)
    E = P*t = m*N  (E=energy)
    */
    return _pulse_count * _power_multiplier / 1000000.0;

}

void BL0937::resetEnergy() {
    _pulse_count = 0;
}

void BL0937::expectedCurrent(double value) {
    if (_current == 0) getCurrent();
    if (_current > 0) _current_multiplier *= (value / _current);
}

void BL0937::expectedVoltage(unsigned int value) {
    if (_voltage == 0) getVoltage();
    if (_voltage > 0) _voltage_multiplier *= ((double) value / _voltage);
}

void BL0937::expectedActivePower(unsigned int value) {
    if (_power == 0) getActivePower();
    if (_power > 0) _power_multiplier *= ((double) value / _power);
}

void BL0937::resetMultipliers() {
    _calculateDefaultMultipliers();
}

void BL0937::setResistors(double current, double voltage_upstream, double voltage_downstream) {
    if (voltage_downstream > 0) {
        _current_resistor = current;
        _voltage_resistor = (voltage_upstream + voltage_downstream) / voltage_downstream;
        _calculateDefaultMultipliers();
    }
}
#if defined LIBRETINY
void /* ICACHE_RAM_ATTR */ BL0937::cf_interrupt() {
#else
void ICACHE_RAM_ATTR BL0937::cf_interrupt() {
#endif
    unsigned long now = micros();
    _power_pulse_width = now - _last_cf_interrupt;
    _last_cf_interrupt = now;
    _pulse_count++;
}
#if defined LIBRETINY
void /* ICACHE_RAM_ATTR */ BL0937::cf1_interrupt() {
#else    
void ICACHE_RAM_ATTR BL0937::cf1_interrupt() {
#endif
    unsigned long now = micros();

    if ((now - _first_cf1_interrupt) > _pulse_timeout) {

        unsigned long pulse_width;
        
        if (_last_cf1_interrupt == _first_cf1_interrupt) {
            pulse_width = 0;
        } else {
            pulse_width = now - _last_cf1_interrupt;
        }

        if (_mode == _current_mode) {
            _current_pulse_width = pulse_width;
        } else {
            _voltage_pulse_width = pulse_width;
        }

        _mode = 1 - _mode;
        digitalWrite(_sel_pin, _mode);
        _first_cf1_interrupt = now;

    }

    _last_cf1_interrupt = now;

}

void BL0937::_checkCFSignal() {
    if ((micros() - _last_cf_interrupt) > _pulse_timeout) _power_pulse_width = 0;
}

void BL0937::_checkCF1Signal() {
    if ((micros() - _last_cf1_interrupt) > _pulse_timeout) {
        if (_mode == _current_mode) {
            _current_pulse_width = 0;
        } else {
            _voltage_pulse_width = 0;
        }
        toggleMode();
    }
}

// These are the multipliers for current, voltage and power as per datasheet
// These values divided by output period (in useconds) give the actual value
void BL0937::_calculateDefaultMultipliers() {
    _current_multiplier = ( 1000000.0 * 512 * V_REF / _current_resistor / 24.0 / F_OSC ) * 0.850112464f;
    _voltage_multiplier = ( 1000000.0 * 512 * V_REF * _voltage_resistor /  2.0 / F_OSC) * 0.863158011f;      
    _power_multiplier =   ( 1000000.0 * 128 * V_REF * V_REF * _voltage_resistor / _current_resistor / 48.0 / F_OSC) * 0.713465334f;
}
