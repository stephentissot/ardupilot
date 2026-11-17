/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or later.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_H723_ADC3_ANALOG_ENABLED
namespace ChibiOS {

class H723AnalogIn;

class H723AnalogSource : public AP_HAL::AnalogSource {
public:
    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t pin) override WARN_IF_UNUSED;
    float voltage_average() override;
    bool voltage_average_healthy(float &voltage) override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    friend class H723AnalogIn;
    float read(bool average) const;
    uint8_t _pin = ANALOG_INPUT_NONE;
};

// EpervierFC wiring only, NOT a generic STM32H723 ADC pin mapping.
class H723AnalogIn : public AP_HAL::AnalogIn {
public:
    // Pool construction does not touch ChibiOS or hardware. Scheduler::init()
    // registers the sole service IO callback; init must not register another.
    void init() override {}
    AP_HAL::AnalogSource *channel(int16_t pin) override;
    bool valid_analog_pin(uint16_t pin) const override;
    float board_voltage() override { return 0.0f; } // 5V rail not measured.

private:
    static constexpr uint8_t MAX_SOURCES = 16;
    H723AnalogSource _sources[MAX_SOURCES];
    uint8_t _num_sources = 0;
};

} // namespace ChibiOS
#endif // HAL_H723_ADC3_ANALOG_ENABLED