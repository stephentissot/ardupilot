/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or later.
 */
#include "H723ADC3Bench.h"

#if HAL_H723_ADC3_BENCH_DIAGNOSTICS || HAL_H723_ADC3_ANALOG_ENABLED
#include <hal.h>
#include <AP_HAL/AP_HAL.h>
#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
#include <GCS_MAVLink/GCS.h>
#endif
#include <stddef.h>
#include <string.h>

#if !defined(STM32H723xx) || !defined(APJ_BOARD_ID) || APJ_BOARD_ID != 9998
#error "H723 ADC3 service is restricted to EpervierFC STM32H723"
#endif
#if HAL_USE_ADC
#error "H723 ADC3 service requires HAL_USE_ADC=FALSE (exclusive ADC ownership)"
#endif
#if defined(STM32_ADC_USE_ADC3) && STM32_ADC_USE_ADC3
#error "H723 ADC3 service requires exclusive ownership (no ChibiOS ADC3 driver)"
#endif
#ifdef HAL_USE_EMPTY_IO
#error "H723 ADC3 service requires scheduler IO callbacks"
#endif

extern const AP_HAL::HAL &hal;

using namespace ChibiOS;

// Verified against local ST stm32h723xx.h and ST HAL/LL ADC (ADC_VER_V5_V90):
// HAL_ADC_Init/ConfigChannel, HAL_ADCEx_Calibration_Start and LL_ADC_Enable.
// https://github.com/STMicroelectronics/stm32h7xx-hal-driver
// This CMSIS revision uses ADC_TypeDef aliases, NOT the H743 register layout.
static_assert(offsetof(ADC_TypeDef, SQR1) == 0x30, "H723 SQR1 layout");
static_assert(offsetof(ADC_TypeDef, DR) == 0x40, "H723 DR layout");
static_assert(offsetof(ADC_TypeDef, LTR2_DIFSEL) == 0xB0, "H723 ADC3 DIFSEL layout");
static_assert(offsetof(ADC_TypeDef, HTR2_CALFACT) == 0xB4, "H723 ADC3 CALFACT layout");
static_assert(ADC3_CFGR_RES_Pos == 3, "H723 ADC3 resolution encoding");
// CKMODE=11 is synchronous HCLK4 /4. PRESC only divides the asynchronous
// source: it cannot implement synchronous /8. Do not change any shared PLL.
static constexpr uint32_t adc_clock_mode = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1;
static_assert(STM32_HCLK >= 4000000U && STM32_HCLK <= 200000000U,
              "Recheck ADC3 clock/timing limits for this HCLK4 (ADC <=50MHz)");
#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
static_assert(HAL_H723_ADC3_BENCH_VBAT_RATIO > 0 &&
              HAL_H723_ADC3_BENCH_VBAT_RATIO <= 100,
              "Bench VBAT divider ratio must be in 1..100");
#endif

// Zero-initialised global BSS; no allocation, constructor or static guard.
H723ADC3Bench H723ADC3Bench::_singleton;

H723ADC3Bench &H723ADC3Bench::instance()
{
    return _singleton;
}

bool H723ADC3Bench::read_sample(uint8_t pin, Sample &sample) const
{
    if (pin != 5 && pin != 9) {
        sample = {};
        return false;
    }
    chSysLock();
    sample = _samples[pin == 5 ? 0 : 1];
    sample.failed = _cache_failed;
    chSysUnlock();
        // 64-bit age cannot revive a stalled cache after a micros() wrap.
    return sample.valid && !sample.failed &&
            AP_HAL::micros64() - sample.timestamp_us < 500000ULL;
}

void H723ADC3Bench::shutdown()
{
    if (!_owns_adc) {
        return;
    }
    // RCC hard reset is the bounded fallback even for a stuck calibration or
    // conversion. Only ADC3 and its own common block are reset, never ADC12.
    chSysLock(); // Protect RCC read/modify/write against other clock users.
    RCC->AHB4RSTR |= RCC_AHB4RSTR_ADC3RST;
    (void)RCC->AHB4RSTR;
    __DSB();
    RCC->AHB4RSTR &= ~RCC_AHB4RSTR_ADC3RST;
    (void)RCC->AHB4RSTR;
    RCC->AHB4ENR &= ~RCC_AHB4ENR_ADC3EN;
    (void)RCC->AHB4ENR;
    chSysUnlock();
    _owns_adc = false;
}

void H723ADC3Bench::fail(const char *reason)
{
    _failure = reason;
    chSysLock();
    _cache_failed = true;
    chSysUnlock();
    shutdown();
    _state = State::Failed; // Sticky: reboot required; no retry loop.
}

#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
void H723ADC3Bench::report(uint32_t now_ms)
{
    if (hal.util->get_soft_armed()) {
        return; // Also suppress optional diagnostics in analog mode while armed.
    }
    if (now_ms - _last_report_ms < 5000U) {
        return;
    }
    _last_report_ms = now_ms;
    if (_state == State::Failed) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ADC3 bench fail: %s", _failure);
        return; // Do not present stale samples as live after a failure.
    }
    if (_state == State::LegacyCancelled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADC3 bench aborted: arm observed");
        return;
    }
    Sample pf3;
    Sample pf4;
    const bool pf3_fresh = read_sample(5, pf3);
    const bool pf4_fresh = read_sample(9, pf4);
    if (!pf3_fresh || !pf4_fresh) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADC3 bench: no complete sample pair");
        return;
    }
    // Assumed VDDA/VREF+=3300mV, not measured with internal VREFINT.
    const uint32_t pf3_mv = (uint32_t(pf3.raw) * 3300U + 2047U) / 4095U;
    const uint32_t pf4_mv = (uint32_t(pf4.raw) * 3300U + 2047U) / 4095U;
    const uint32_t vbat_mv = (uint32_t(pf3.raw) * 3300U *
                              HAL_H723_ADC3_BENCH_VBAT_RATIO + 2047U) / 4095U;
    // Two separate cached STATUSTEXTs, each <50 chars at maximum values.
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADC3 PF3 raw=%u mV=%lu VBAT=%lumV",
                  unsigned(pf3.raw), (unsigned long)pf3_mv, (unsigned long)vbat_mv);
    // CUR transfer function/zero offset unknown: voltage only, never amps.
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ADC3 PF4 raw=%u mV=%lu (no amps)",
                  unsigned(pf4.raw), (unsigned long)pf4_mv);
}
#endif

void H723ADC3Bench::update()
{
#if !HAL_H723_ADC3_ANALOG_ENABLED
    // IO thread only. Once observed, arm permanently cancels this boot's test.
    // Reset aborts any outstanding one-shot/calibration without polling a stop.
    if (hal.util->get_soft_armed()) {
        shutdown();
        if (_state != State::Failed) {
            _state = State::LegacyCancelled;
        }
        chSysLock();
        _samples[0].valid = false;
        _samples[1].valid = false;
        chSysUnlock();
        return; // No telemetry while armed, no automatic restart on disarm.
    }
#endif

    const uint32_t now_us = AP_HAL::micros();
    const uint32_t elapsed_us = now_us - _state_us;
    switch (_state) {
    case State::Start:
        // APJ IDs are local, so also check the actual board name before MMIO.
        if (strcmp(CHIBIOS_BOARD_NAME, "EpervierFC") != 0 &&
            strcmp(CHIBIOS_BOARD_NAME, "EpervierFC_IMU_Bench") != 0) {
            fail("wrong board");
            break;
        }
        // hwdef reserves these dedicated inputs without enabling the
        // incompatible ChibiOS ADCv4 driver. Switch only these two pins;
        // PAL_MODE_INPUT_ANALOG also clears their pull resistors.
        palSetPadMode(GPIOF, 3, PAL_MODE_INPUT_ANALOG);
        palSetPadMode(GPIOF, 4, PAL_MODE_INPUT_ANALOG);
        if ((GPIOF->MODER & ((3U << 6) | (3U << 8))) != ((3U << 6) | (3U << 8)) ||
            (GPIOF->PUPDR & ((3U << 6) | (3U << 8))) != 0) {
            fail("PF3/PF4 not analog");
            break;
        }
        chSysLock();
        if ((RCC->AHB4ENR & RCC_AHB4ENR_ADC3EN) != 0) {
            chSysUnlock();
            fail("already owned"); // Never reset an existing owner's ADC.
            break;
        }
        RCC->AHB4ENR |= RCC_AHB4ENR_ADC3EN;
        (void)RCC->AHB4ENR; // ST RCC enable/readback delay.
        _owns_adc = true;
        RCC->AHB4RSTR |= RCC_AHB4RSTR_ADC3RST;
        (void)RCC->AHB4RSTR;
        __DSB();
        RCC->AHB4RSTR &= ~RCC_AHB4RSTR_ADC3RST;
        (void)RCC->AHB4RSTR;
        chSysUnlock();
        ADC3->CR = 0; // Exit DEEPPWD first, with ADEN=0.
        ADC3_COMMON->CCR = adc_clock_mode; // PRESC=0; ALL internal paths off.
        ADC3->IER = 0;
        ADC3->CFGR = 0; // ADC3 RES[4:3]=00: 12-bit, right, SW, single, no DMA.
        ADC3->CFGR2 = 0; // No oversampling, bulb or triggered sampling.
        ADC3->LTR2_DIFSEL = 0; // ADC3 single-ended at 0xB0 (not ADC12 DIFSEL).
        // SMP=111 means 640.5 cycles for H723 ADC3, not ADC12's 810.5.
        ADC3->SMPR1 = ADC_SMPR1_SMP5 | ADC_SMPR1_SMP9;
        // ADC3 has NO PCSEL: 0x1C is reserved. Reset SQR length is one rank.
        ADC3->SQR1 = 5U << ADC_SQR1_SQ1_Pos;
        ADC3->CR = ADC_CR_ADVREGEN;
        _state_us = AP_HAL::micros();
        _state = State::Regulator;
        break;

    case State::Regulator:
        if (elapsed_us < 20U) {
            break; // Asynchronous regulator settling (ST LL requires >=10us).
        }
        if ((ADC3->CR & (ADC_CR_ADVREGEN | ADC_CR_DEEPPWD)) != ADC_CR_ADVREGEN ||
            ADC3_COMMON->CCR != adc_clock_mode) {
            fail("regulator/clock");
            break;
        }
        // Offset calibration, single-ended; no ADC12-only BOOST/ADCALLIN.
        ADC3->CR = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
        _state_us = AP_HAL::micros();
        _state = State::Calibrating;
        break;

    case State::Calibrating:
        if ((ADC3->CR & ADC_CR_ADCAL) == 0) {
            _state_us = now_us;
            _state = State::CalibrationSettling;
        } else if (elapsed_us >= 100000U) {
            fail("calibration timeout");
        }
        break;

    case State::CalibrationSettling:
        if (elapsed_us >= 20U) { // >4 ADC clocks even at minimum allowed HCLK.
            ADC3->ISR = ADC_ISR_ADRDY;
            ADC3->CR = ADC_CR_ADVREGEN | ADC_CR_ADEN;
            _state_us = AP_HAL::micros();
            _state = State::Enabling;
        }
        break;

    case State::Enabling:
        if ((ADC3->ISR & ADC_ISR_ADRDY) != 0) {
            _state = State::Idle;
            _last_sample_us = now_us;
        } else if (elapsed_us >= 10000U) {
            fail("ADRDY timeout");
        }
        break;

    case State::Idle:
        // One conversion/25ms, alternating PF3 INP5/PF4 INP9: 20Hz per pin.
        if (now_us - _last_sample_us >= 25000U) {
            if ((ADC3->CR & (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART |
                             ADC_CR_ADSTP | ADC_CR_ADDIS | ADC_CR_ADCAL)) != ADC_CR_ADEN) {
                fail("unexpected ADC state");
                break;
            }
            ADC3->SQR1 = (_channel_index == 0 ? 5U : 9U) << ADC_SQR1_SQ1_Pos;
            ADC3->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
            _conversion_start_us = AP_HAL::micros64();
            ADC3->CR = ADC_CR_ADVREGEN | ADC_CR_ADSTART;
            _state_us = AP_HAL::micros();
            _last_sample_us = _state_us;
            _state = State::Converting;
        }
        break;

    case State::Converting:
        if ((ADC3->ISR & ADC_ISR_OVR) != 0) {
            fail("overrun");
        } else if ((ADC3->ISR & (ADC_ISR_EOC | ADC_ISR_EOS)) == (ADC_ISR_EOC | ADC_ISR_EOS) &&
                   (ADC3->CR & ADC_CR_ADSTART) == 0) {
            const uint16_t raw = ADC3->DR & 0xFFFU; // Read clears EOC.
            ADC3->ISR = ADC_ISR_EOS;
            // Bounded O(1) integer EMA. Only the IO callback writes this cache;
            // readers and writer use the same short lock, with no FP or MMIO.
            chSysLock();
            Sample &sample = _samples[_channel_index];
            const int32_t target_q8 = int32_t(raw) * 256;
            if (sample.valid) {
                sample.average_q8 = int32_t(sample.average_q8) +
                                    (target_q8 - int32_t(sample.average_q8)) / 8;
            } else {
                sample.average_q8 = target_q8;
            }
            sample.raw = raw;
            // Age from the conversion start, not this IO callback: a delayed
            // readout must not turn an old completed conversion into fresh data.
            sample.timestamp_us = _conversion_start_us;
            sample.valid = true;
            chSysUnlock();
            _channel_index ^= 1U;
            _state = State::Idle;
        } else if (elapsed_us >= 10000U) {
            fail("EOC timeout");
        }
        break;

    case State::Failed:
    case State::LegacyCancelled:
        break;
    }
#if HAL_H723_ADC3_BENCH_DIAGNOSTICS
    report(AP_HAL::millis());
#endif
}
#endif // HAL_H723_ADC3_BENCH_DIAGNOSTICS || HAL_H723_ADC3_ANALOG_ENABLED