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

#if HAL_H723_ADC3_BENCH_DIAGNOSTICS || HAL_H723_ADC3_ANALOG_ENABLED
#include <stdint.h>

#ifndef HAL_H723_ADC3_BENCH_VBAT_RATIO
#define HAL_H723_ADC3_BENCH_VBAT_RATIO 11
#endif

namespace ChibiOS {

// Exclusive board-only ADC3 owner. Hardware is advanced only by the IO thread.
class H723ADC3Bench {
public:
    static H723ADC3Bench &instance();
    void update();

    struct Sample {
        uint16_t raw;
        uint32_t average_q8; // ADC counts * 256, EMA with alpha=1/8.
        uint64_t timestamp_us;
        bool valid;
        bool failed;
    };

    // Thread context only; coherent snapshot under a short system lock.
    // Returns true only for a valid, nonfailed sample younger than 500ms.
    // Logical IDs are EpervierFC-only: 5=PF3 VBAT, 9=PF4 CUR.
    bool read_sample(uint8_t pin, Sample &sample) const;

private:
    static H723ADC3Bench _singleton;

    enum class State : uint8_t {
        Start, Regulator, Calibrating, CalibrationSettling, Enabling,
        Idle, Converting, Failed, LegacyCancelled
    };

    void fail(const char *reason);
    void shutdown();
#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
    void report(uint32_t now_ms);
#endif

    State _state;
    bool _owns_adc;
    uint8_t _channel_index;
    Sample _samples[2];
    bool _cache_failed; // Sticky; protected along with _samples by chSysLock.
    uint32_t _state_us;
    uint32_t _last_sample_us;
    uint64_t _conversion_start_us;
#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
    uint32_t _last_report_ms;
#endif
    const char *_failure;
};

}
#endif // HAL_H723_ADC3_BENCH_DIAGNOSTICS || HAL_H723_ADC3_ANALOG_ENABLED