#ifndef RTC_H
#define RTC_H

#include <stdbool.h>

typedef uint64_t millis_t;

void rtc_init(void);

uint32_t rtc_get_counter(void);

uint32_t rtc_get_hours(void);

millis_t rtc_get_millis(void);

bool rtc_time_ellapsed(millis_t timestamp, uint32_t millis);

#endif