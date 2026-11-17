/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or later.
 */
#include "H723AnalogIn.h"

#if HAL_H723_ADC3_ANALOG_ENABLED
#include <hal.h>
#include "H723ADC3Bench.h"

#if HAL_USE_ADC
#error "H723 AnalogIn requires HAL_USE_ADC=FALSE (exclusive ADC ownership)"
#endif

using namespace ChibiOS;

// Assumed VDDA/VREF+=3.3V, 12-bit ADC3. Divider and current calibration belong
// to consumers (e.g. BATT_VOLT_MULT), never to this pin-voltage adapter.
static constexpr float volts_per_count = 3.3f / 4095.0f;

bool H723AnalogIn::valid_analog_pin(uint16_t pin) const
{
    return pin == 5 || pin == 9; // PF3 VBAT, PF4 CUR on this board only.
}

AP_HAL::AnalogSource *H723AnalogIn::channel(int16_t pin)
{
    // Battery/frontend constructors may request disabled sources before any
    // samples exist. Keep independent, stable handles even for the same pin.
    if (pin != -1 && pin != ANALOG_INPUT_NONE && !valid_analog_pin(pin)) {
        return nullptr;
    }
    chSysLock();
    if (_num_sources == MAX_SOURCES) {
        chSysUnlock();
        return nullptr;
    }
    H723AnalogSource *source = &_sources[_num_sources++];
    source->_pin = pin == -1 ? ANALOG_INPUT_NONE : uint8_t(pin);
    chSysUnlock();
    return source;
}

bool H723AnalogSource::set_pin(uint8_t pin)
{
    const bool supported = pin == 5 || pin == 9;
    chSysLock();
    // Bind even before samples arrive; invalid/disabled pins are fail-closed.
    _pin = supported ? pin : ANALOG_INPUT_NONE;
    chSysUnlock();
    if (!supported) {
        return false;
    }
    H723ADC3Bench::Sample sample;
    // Deliberately recheck even an unchanged pin: health is not pin validity.
    return H723ADC3Bench::instance().read_sample(pin, sample);
}

float H723AnalogSource::read(bool average) const
{
    chSysLock();
    const uint8_t pin = _pin;
    chSysUnlock();
    if (pin == ANALOG_INPUT_NONE) {
        return 0.0f;
    }
    H723ADC3Bench::Sample sample;
    if (!H723ADC3Bench::instance().read_sample(pin, sample)) {
        // No NaNs (battery consumers do arithmetic even when unhealthy), and
        // never present a stale or failed cached measurement as a live value.
        return 0.0f;
    }
    // Cache copy and all floating-point conversion are outside the system lock.
    return average ? float(sample.average_q8) / 256.0f : float(sample.raw);
}

float H723AnalogSource::read_average()
{
    return read(true);
}

float H723AnalogSource::read_latest()
{
    return read(false);
}

float H723AnalogSource::voltage_average()
{
    return read_average() * volts_per_count;
}

float H723AnalogSource::voltage_latest()
{
    return read_latest() * volts_per_count;
}

bool H723AnalogSource::voltage_average_healthy(float &voltage)
{
    chSysLock();
    const uint8_t pin = _pin;
    chSysUnlock();
    H723ADC3Bench::Sample sample;
    const bool healthy = H723ADC3Bench::instance().read_sample(pin, sample);
    voltage = healthy ? float(sample.average_q8) * (volts_per_count / 256.0f) : 0.0f;
    return healthy;
}

float H723AnalogSource::voltage_average_ratiometric()
{
    // No measured VCC: ratiometric compensation is unavailable.
    return voltage_average();
}
#endif // HAL_H723_ADC3_ANALOG_ENABLED