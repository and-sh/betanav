/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
//#include "drivers/sensor.h"

timerHardware_t timerHardware[] = {
    DEF_TIM(TIM3, CH1, PB4, TIM_USE_OUTPUT_AUTO, 0, 0), // S1
    DEF_TIM(TIM3, CH2, PB5, TIM_USE_OUTPUT_AUTO, 0, 0), // S2
    //DEF_TIM(TIM1, CH2N, PB0, TIM_USE_OUTPUT_AUTO, 0, 0), // S3 - Timer excluded as used by LED
    //DEF_TIM(TIM3, CH3, PB0, TIM_USE_OUTPUT_AUTO, 0, 0), // S3
    DEF_TIM(TIM8, CH2N, PB0, TIM_USE_OUTPUT_AUTO, 0, 0), // S3
    //DEF_TIM(TIM1, CH3N, PB1, TIM_USE_OUTPUT_AUTO, 0, 0), // S4 - Timer excluded as used by LED
    //DEF_TIM(TIM3, CH4, PB1, TIM_USE_OUTPUT_AUTO, 0, 0), // S4
    DEF_TIM(TIM8, CH3N, PB1, TIM_USE_OUTPUT_AUTO, 0, 0), // S4

    DEF_TIM(TIM16, CH1N, PB6, TIM_USE_OUTPUT_AUTO, 0, 0), // S5
    //DEF_TIM(TIM4, CH1, PB6, TIM_USE_OUTPUT_AUTO, 0, 0), // - Timer excluded as used by BEEPER
    DEF_TIM(TIM17, CH1N, PB7, TIM_USE_OUTPUT_AUTO, 0, 0), // S6
    //DEF_TIM(TIM4, CH2, PB7, TIM_USE_OUTPUT_AUTO, 0, 0), // - Timer excluded as used by BEEPER
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_OUTPUT_AUTO, 0, 0), // S7
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_OUTPUT_AUTO, 0, 0), // S8

    DEF_TIM(TIM4, CH4, PD15, TIM_USE_BEEPER, 0, 0),
    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED, 0, 0),

};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
