/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or later.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_H723_ADC3_BENCH_DIAGNOSTICS
#define HAL_H723_ADC3_BENCH_DIAGNOSTICS 0
#endif

#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
#include <stdint.h>

#ifndef HAL_H723_ADC3_BENCH_VBAT_RATIO
#define HAL_H723_ADC3_BENCH_VBAT_RATIO 11
#endif

namespace ChibiOS {

// Exclusive ADC3 owner; bench telemetry only, not an AnalogIn backend.
class H723ADC3Bench {
public:
    static H723ADC3Bench &instance();
    void update();

private:
    static H723ADC3Bench _singleton;

    enum class State : uint8_t {
        Start, Regulator, Calibrating, CalibrationSettling, Enabling,
        Idle, Converting, Failed, Cancelled
    };

    void fail(const char *reason);
    void shutdown();
    void report(uint32_t now_ms);

    State _state;
    bool _owns_adc;
    uint8_t _channel_index;
    uint8_t _valid_mask;
    uint16_t _raw[2];
    uint32_t _state_us;
    uint32_t _last_sample_us;
    uint32_t _last_report_ms;
    const char *_failure;
};

}
#endif // HAL_H723_ADC3_BENCH_DIAGNOSTICS