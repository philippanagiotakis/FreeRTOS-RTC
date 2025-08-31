// lptimTick.c -- Jeff Tenney
//
// STM32 No-Drift FreeRTOS Tick/Tickless via LPTIM
//
// Revision: 2021.11.23
// Tabs: None
// Columns: 110
// Compiler: gcc (GNU) / armcc (Arm-Keil) / iccarm (IAR)
// SPDX-License-Identifier: MIT


// Copyright 2021 Jeff Tenney <jeff.tenney@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
// LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
// EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
// USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx.h"
#include "main.h"
#include "stm32h7xx_hal_rtc_ex.h"

#ifndef RTC_WAKEUPCLOCK_RTCCLK_DIV32
#define RTC_WAKEUPCLOCK_RTCCLK_DIV32    ((uint32_t)0x00000000U)
#endif


extern RTC_HandleTypeDef hrtc;
//      This FreeRTOS port "extension" for STM32 uses LPTIM to generate the OS tick instead of the systick
// timer.  The benefit of the LPTIM is that it continues running in "stop" mode as long as its clock source
// does.  A typical clock source for the LPTIM is LSE (or LSI), which does keep running in stop mode.
//
//      The resulting FreeRTOS port:
//
//   o Allows use of low-power stop modes during tickless idle, while still keeping kernel time.
//   o Eliminates kernel-time drift associated with tickless idle in official FreeRTOS port
//   o Eliminates kernel-time drift caused by rounding the OS tick to a whole number of timer counts.
//   o Avoids drift and errors found in other LPTIM implementations available from ST or the public domain.
//   o Detects/reports ticks dropped due to the application masking interrupts (the tick ISR) for too long.
//
//      This software is currently adapted for STM32L4(+) but is easily adaptable to (or already compatible
// with) any STM32 that provides an LPTIM peripheral, such as STM32L, STM32F, STM32G, STM32H, STM32W, and the
// new STM32U.


// Terminology
//
//      "Count" - What a timer does in its "count" register (CNT).
//
//      "Tick" - The OS tick, made up of some number of timer counts.


// Perfect Tick Frequency
//
//      This software optionally varies the number of timer counts per OS tick to achieve the target OS tick
// frequency.  The OS tick generally occurs no more than half a timer count early or half a timer count late
// compared to the ideal tick time.  This is especially useful with a 32768Hz reference on LSE and a desired
// 1000Hz system tick.  In that case, this software uses 32-count and 33-count tick durations as needed to
// stay on the 1000Hz tick schedule.  No matter how you set configLPTIM_REF_CLOCK_HZ and configTICK_RATE_HZ,
// this software stays precisely on schedule.
//
//      You can disable this feature and instead use a simple, constant number of timer counts per OS tick by
// defining configLPTIM_ENABLE_PRECISION to 0.  Your effective tick rate will be as close as possible to
// configTICK_RATE_HZ while using a constant number of counts per tick.  For example, with a 32768 Hz clock
// and an ideal tick frequency of 1000 Hz, the actual tick frequency is ~993 Hz.


// Silicon Bug
//
//      The LPTIM has a silicon bug that can cause the CPU to be temporarily stuck in the LPTIM ISR with no
// way for the CPU to clear the IRQ.  The silicon bug is not documented by ST (yet), but the "stuck" condition
// appears to last one full count of the LPTIM.  It seems to occur only when the application is using the MCU
// "stop" power modes.
//
//      Any attempt to use stop mode in the window between a "match" and the CMPM one count later causes the
// match interrupt to be asserted (early), and it cannot be cleared until the CMPM flag is set.  Additionally,
// any attempt to use stop mode during the timer count after a CMPM event that we tried to suppress too late
// also results in the entire count duration stuck in the ISR.  By "suppress too late", we mean write a new
// value to CMP just before a CMPM event.
//
//      As a workaround, the application should be sure that configTICK_INTERRUPT_PRIORITY is a lower priority
// than application interrupts.  We considered a workaround in this file but realized that our only viable
// recourse is to avoid stop mode at strategic times, but those time windows last several timer counts.  Using
// sleep mode instead of stop mode for several counts is more costly than the silicon bug itself, which uses
// run mode instead of stop mode for one timer count (stuck in interrupt handlers).


// Quirks of LPTIM
//
//      The following "quirks" indicate that LPTIM is designed for PWM output control and not for generating
// timed interrupts.  This software overcomes all of them.
//
//   o Writes to CMP are delayed by a sync mechanism inside the timer.  The sync takes ~3 timer counts.
//   o During the synchronization process, additional writes to CMP are prohibited.
//   o CMPOK (sync completion) comes *after* the CMP value is asynchronously available for match events.
//   o The match condition is not simply "CNT == CMP" but is actually "CNT >= CMP && CNT != ARR".
//   o The timer sets CMPM (and optional IRQ) one timer count *after* the match condition is reached.
//   o With a new CMP value, the timer must first be in a "no-match" condition to generate a match event.
//   o Setting CMP == ARR is prohibited.  See below.
//   o Changing IER (interrupt-enable register) is prohibited while LPTIM is enabled.
//   o The CPU can get stuck temporarily in the LPTIM ISR when using stop mode.  See "Silicon Bug" above.
//
//      This software sets ARR to 0xFFFF permanently and modifies CMP to arrange each next tick interrupt.
// Due to the rule against setting CMP == ARR, we never set CMP to 0xFFFF.  If the ideal CMP value for a tick
// interrupt is 0xFFFF, we use 0 instead.


// Side Effects
//
//   o Tick Overhead.  OS ticks generated by this software have more overhead than ticks generated by the
//     official port code.  In most applications, the overhead doesn't make any real difference.  Our tick ISR
//     is longer by a handful of CPU instructions, and we execute a 2nd, very short ISR in between ticks.
//
//   o Tick IRQ Priority.  The tick IRQ priority must be high enough that no combination of ISRs can block its
//     execution for longer than 1 tick.  See configTICK_INTERRUPT_PRIORITY (below) for more information.
//     However, the application may safely mask interrupts for longer than 1 tick, rare as that need may be.
//     (One common example is for "fast-programming" flash memory on STM32.)  In that case, this software even
//     reports dropped ticks afterward via traceTICKS_DROPPED().
//
//   o Tick Jitter.  When precision is enabled (configLPTIM_ENABLE_PRECISION), ticks generated by this
//     software have jitter, as described above in "Perfect Tick Frequency".  In most applications, jitter in
//     the tick periods is not a concern.


#if ( !defined(configUSE_TICKLESS_IDLE) || configUSE_TICKLESS_IDLE != 2 )
#warning Please edit FreeRTOSConfig.h to define configUSE_TICKLESS_IDLE as 2 *or* exclude this file.
#else

#ifdef xPortSysTickHandler
#warning Please edit FreeRTOSConfig.h to eliminate the preprocessor definition for xPortSysTickHandler.
#endif


//      Symbol configTICK_INTERRUPT_PRIORITY, optionally defined in FreeRTOSConfig.h, controls the tick
// interrupt priority.  Most applications should define configTICK_INTERRUPT_PRIORITY to be
// configLIBRARY_LOWEST_INTERRUPT_PRIORITY because the tick doesn't need a high priority.  But if your
// application has long ISRs, you may need to increase the priority of the tick.  The tick priority must be
// high enough that no combination of ISRs at or above its priority level can block the tick ISR for longer
// than 1 tick.  But the priority must not be higher than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
// (meaning that the value of configTICK_INTERRUPT_PRIORITY must not be numerically lower).
//
//      Be sure to consider the information in "Silicon Bug" above before increasing the interrupt priority of
// the system tick.
//
#ifndef configTICK_INTERRUPT_PRIORITY
#define configTICK_INTERRUPT_PRIORITY configLIBRARY_LOWEST_INTERRUPT_PRIORITY
// default only; see above
#endif

//      Symbol configTICK_USES_LSI, optionally defined in FreeRTOSConfig.h, is defined only when LPTIM should
// use LSI as the clock instead of LSE.  By default, however, this software configures LPTIM to use LSE
// because a key feature of this software is timing accuracy -- no drift in tickless idle.
//
#ifdef configTICK_USES_LSI
   #define LPTIMSEL_Val 1 // LSI
   #define IS_REF_CLOCK_READY() (RCC->CSR & RCC_CSR_LSIRDY)
#else
   #define LPTIMSEL_Val RCC_LPTIM2CLKSOURCE_LSE // LSE
   #define IS_REF_CLOCK_READY() (RCC->BDCR & RCC_BDCR_LSERDY)
#endif

//      Symbol configLPTIM_REF_CLOCK_HZ, optionally defined in FreeRTOSConfig.h, is the frequency of the
// selected reference clock or source clock for LPTIM.  If configTICK_USES_LSI is defined, then
// configLPTIM_REF_CLOCK_HZ equals the frequency of LSI (typically 32000 Hz or 37000 Hz depending on the MCU).
// Otherwise, configLPTIM_REF_CLOCK_HZ equals the frequency of LSE (usually 32768 Hz).
//
#ifndef configLPTIM_REF_CLOCK_HZ
#define configLPTIM_REF_CLOCK_HZ 32768UL
#endif

//      Symbol configLPTIM_ENABLE_PRECISION, optionally defined in FreeRTOSConfig.h, allows a configuration to
// eliminate the arithmetic in this software that maintains configTICK_RATE_HZ with perfect precision, as
// described above in "Perfect Tick Frequency".  The arithmetic corrects for any error in rounding the desired
// tick duration to a whole number of timer counts.  No matter how you set the precision option, this software
// eliminates the drift normally associated with tickless idle.  The precision option is enabled by default
// because it has a very small footprint by all measures (flash, RAM, execution time).  However, because the
// precision feature requires additional division operations, Cortex M0 users may consider disabling it.  CM0
// does not have a native divide instruction, so division operations are a little slow on that platform.
//
#ifndef configLPTIM_ENABLE_PRECISION
#define configLPTIM_ENABLE_PRECISION 0
#endif

//      If the application masks interrupts (specifically the tick interrupt) long enough to drop a tick, then
// the tick ISR calls this trace macro to report the condition along with the number of ticks dropped.  You
// can define this macro in FreeRTOSConfig.h.  Remember that the macro executes from within the tick ISR.
// Also be aware that the ISR resets the phase of the ticks after calling this macro.
//
//      If ticks are dropped and you did *not* mask the tick interrupt long enough to drop a tick, you
// probably have ISRs blocking the tick ISR for too long.  This is a serious issue that requires changes to
// your design.  Please see configTICK_INTERRUPT_PRIORITY above.
//
#ifndef traceTICKS_DROPPED
#define traceTICKS_DROPPED(x)
#endif

#define LPTIM_CLOCK_HZ ( configLPTIM_REF_CLOCK_HZ )

#define RTC_WAKEUP_CLOCK_DIV RTC_WAKEUPCLOCK_RTCCLK_DIV16  // ~2048 Hz if LSE = 32.768 kHz
#define RTC_CLOCK_HZ         32768
#define RTC_DIVIDER          16
#define WAKEUP_CLOCK_HZ      (RTC_CLOCK_HZ / RTC_DIVIDER)

static TickType_t xMaximumSuppressedTicks;      //   We won't try to sleep longer than this many ticks during
                                                // tickless idle because any longer might confuse the logic in
                                                // our implementation.

static uint32_t ulTimerCountsForOneTick;        //   A "baseline" tick has this many timer counts.  The
                                                // baseline tick is as close as possible to the ideal duration
                                                // but is a whole number of timer counts.

#if ( configLPTIM_ENABLE_PRECISION != 0 )

   static int lSubcountErrorPerTick;            //   A "baseline" tick has this much error, measured in timer
                                                // subcounts.  There are configTICK_RATE_HZ subcounts per
                                                // count.  When this field is negative, the baseline tick is a
                                                // little too long because we rounded "up" to the nearest
                                                // whole number of counts per tick.  When this field is
                                                // positive, the baseline tick is a little too short because
                                                // we rounded "down" to the nearest whole number of counts per
                                                // tick.

   static volatile int lRunningSubcountError;   //   This error accumulator never exceeds half a count, or
                                                // configTICK_RATE_HZ/2.  When this field is negative, the
                                                // next tick is slightly late; when this field is positive,
                                                // the next tick is slightly early.  This field allows us to
                                                // schedule each tick on the timer count closest to the ideal
                                                // tick time.
#endif // configLPTIM_ENABLE_PRECISION

static volatile uint16_t usIdealCmp;            //   This field doubles as a write cache for LPTIM->CMP and a
                                                // way to remember that we set CMP to 0 because 0xFFFF isn't
                                                // allowed (hardware limitation).

static volatile uint8_t isCmpWriteInProgress;   //   This field helps us remember when we're waiting for the
                                                // CMP write to finish.  We must not write to CMP while a
                                                // previous write is still in progress.

static volatile uint8_t isTickNowSuppressed;    //   This field helps the tick ISR determine whether
                                                // usIdealCmp is in the past or the future.


// LPTIM Instance Selection
//
//      If your MCU has multiple LPTIM instances, you must (1) select the instance you want this software to
// use for the OS tick by updating the the following three #defines, and (2) change the first five statements
// of vPortSetupTimerInterrupt() to match your selection.  The default configuration is for LPTIM1 because it
// operates even in the lowest-power STOP level.
//
//      If your MCU has only one LPTIM instance, you may or may not need to update these three #defines.  But
// you must change the first five statements of vPortSetupTimerInterrupt() to match your STM32.
//
#ifndef LPTIM
#define LPTIM              LPTIM2
#define LPTIM_IRQn         LPTIM2_IRQn
#define LPTIM_IRQHandler   LPTIM2_IRQHandler
#endif

//============================================================================================================
// vPortSetupTimerInterrupt()
//
//      This function overrides the "standard" port function, decorated with __attribute__((weak)), in port.c.
// Call with interrupts masked.
//
void vPortSetupTimerInterrupt(void)
{
    //   Assumes hrtc is already initialized via MX_RTC_Init()
    // Stop any previously running wakeup timer
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    // Setup RTC to trigger interrupt every 1ms
    //  32768 Hz / 32 = 1024 Hz => 1 tick ~ 1.024 ms
    HAL_RTCEx_SetWakeUpTimer_IT(
        &hrtc,
        1,  // 1 tick for ~1ms tick
        RTC_WAKEUPCLOCK_RTCCLK_DIV32
    );
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    //  Call the FreeRTOS tick handler
    xPortSysTickHandler();
}
//============================================================================================================
// vPortSuppressTicksAndSleep()
//
//      This function overrides the "official" port function, decorated with __attribute__((weak)), in port.c.
// The idle task calls this function with the scheduler suspended, and only when xExpectedIdleTime is >= 2.
//
//      FreeRTOS version 10.4.0 or newer is recommended to ensure this function doesn't potentially return one
// OS tick *after* the intended time.
//


void vPortSuppressTicksAndSleep(TickType_t expectedIdleTime)
{
    uint32_t ticksBefore, ticksAfter;
    uint32_t wakeupTime = 0;

    //Minimum ticks to sleep (FreeRTOS uses this check)
    if (expectedIdleTime < 2) return;

    // Stop the SysTick interrupt
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    //Check if sleep is still OK (e.g. a task became ready)
    if (eTaskConfirmSleepModeStatus() != eAbortSleep)
    {
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        return;
    }

    //Convert expectedIdleTime to wakeup counter ticks
    TickType_t actualSleepTicks = expectedIdleTime;
    wakeupTime = (actualSleepTicks * WAKEUP_CLOCK_HZ) / configTICK_RATE_HZ;
    if (wakeupTime > 0xFFFF)
    {
        wakeupTime = 0xFFFF;//RTC wakeup timer is 16-bit
        actualSleepTicks = (wakeupTime * configTICK_RATE_HZ) / WAKEUP_CLOCK_HZ;
    }

    ticksBefore = xTaskGetTickCount();
    
    RTC_TimeTypeDef now;
    RTC_DateTypeDef date;
    RTC_AlarmTypeDef alarm;
    
    HAL_RTC_GetTime(&hrtc, &now, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
    
    // Convert xExpectedIdleTime from ticks to seconds
    uint32_t sleepSeconds = expectedIdleTime / configTICK_RATE_HZ;
    
    uint8_t alarmSeconds = (now.Seconds + sleepSeconds) % 60;
    uint8_t alarmMinutes = (now.Minutes + ((now.Seconds + sleepSeconds) / 60)) % 60;
    uint8_t alarmHours = now.Hours;
    
    // Configure alarm structure
    alarm.AlarmTime.Hours = alarmHours;
    alarm.AlarmTime.Minutes = alarmMinutes;
    alarm.AlarmTime.Seconds = alarmSeconds;
    alarm.AlarmTime.SubSeconds = 0;
    alarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    alarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    
    alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES; 
    alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    alarm.AlarmDateWeekDay = 1; 
    alarm.Alarm = RTC_ALARM_A;
    

    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
    HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
    
    HAL_SuspendTick();
    HAL_PWREx_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI, PWR_D1_DOMAIN);
    HAL_ResumeTick();

    
    ticksAfter = xTaskGetTickCount();

    vTaskStepTick(actualSleepTicks);
    
    // Restart SysTick
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


#endif  // configUSE_TICKLESS_IDLE == 2
