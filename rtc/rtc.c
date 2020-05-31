#include <stdbool.h>
#include <math.h>
#include "nrf52.h"
#include "rtc.h"

// 1 hour = 3600000 ms


#define RTC (NRF_RTC0)

#define FREQ_TO_PRESQ_CALC(x) round(32768 / x) - 1


typedef enum
{
    RTC_INT_TICK        = RTC_INTENSET_TICK_Pos,
    RTC_INT_OVRFLW      = RTC_INTENSET_OVRFLW_Pos,
    RTC_INT_COMPARE0    = RTC_INTENSET_COMPARE0_Pos,
    RTC_INT_COMPARE1    = RTC_INTENSET_COMPARE1_Pos,
    RTC_INT_COMPARE2    = RTC_INTENSET_COMPARE2_Pos,
    RTC_INT_COMPARE3    = RTC_INTENSET_COMPARE3_Pos,
}RTCInterrupt;


//! TASKS
static void task_start(void);
static void task_stop(void);
static void task_clear(void);

//! EVENTS
static void event_clear_tick(void);

//! Interrupts
static void interrupt_enable(RTCInterrupt option);
static void interrupt_disable(RTCInterrupt option);

//! Compare registers
static void compare_register_set(uint8_t id, uint32_t value);


static volatile uint32_t hours_counter;


static void task_start(void)
{
    RTC->TASKS_START = 1U;
}

static void task_stop(void)
{
    RTC->TASKS_STOP = 1U;
}

static void task_clear(void)
{
    RTC->TASKS_CLEAR = 1U;
}


static void event_clear_tick(void)
{
    RTC->EVENTS_TICK = 0U;
}

static bool event_get_tick(void)
{
    return (bool)RTC->EVENTS_TICK;
}

static void interrupt_enable(RTCInterrupt option)
{
    RTC->INTENSET |= (1 << option);
}

static void interrupt_disable(RTCInterrupt option)
{
    RTC->INTENCLR &= ~(1 << option);
}

static void compare_register_set(uint8_t id, uint32_t value)
{
    if (id < 0 || id > 3)
    {
        return;
    }
    
    RTC->CC[id] = value;
}


void rtc_init(void)
{
    task_stop();

    RTC->PRESCALER = (uint32_t)FREQ_TO_PRESQ_CALC(1000); // 1000 Hz
    compare_register_set(0, (uint32_t)3600000);
    interrupt_enable(RTC_INT_COMPARE0);
    NVIC_ClearPendingIRQ(RTC0_IRQn);
    NVIC_EnableIRQ(RTC0_IRQn);
    task_clear();
    task_start();
}

uint32_t rtc_get_counter(void)
{
    return (uint32_t)RTC->COUNTER;
}

millis_t rtc_get_millis(void)
{
    return (millis_t)hours_counter * (millis_t)3600000 + (millis_t)RTC->COUNTER;
}

uint32_t rtc_get_hours(void)
{
    return hours_counter;
}

bool rtc_time_ellapsed(millis_t timestamp, uint32_t millis)
{
    if (rtc_get_millis() > timestamp + millis)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}


void RTC0_IRQHandler(void)
{
    if (RTC->EVENTS_COMPARE[0] == true)
    {
        RTC->TASKS_CLEAR = 1U;
        RTC->EVENTS_COMPARE[0] = false;
        hours_counter++;
    }
}