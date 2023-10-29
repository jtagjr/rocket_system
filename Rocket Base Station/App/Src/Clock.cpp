
#include "Clock.h"

extern "C"
{
extern RTC_HandleTypeDef hrtc;
}

//------------------------------------------------------------------
// GetInstance
//------------------------------------------------------------------
Clock& Clock::GetInstance()
{
    static Clock instance;
    return instance;
}

//------------------------------------------------------------------
// Constructor
//------------------------------------------------------------------
Clock::Clock()
{
    ClearTime();
}

//------------------------------------------------------------------
// SetTime
//------------------------------------------------------------------
bool Clock::SetTime(time_t eTime)
{
    epochTime = eTime;

    // convert to RTC format
    RTC_DateTypeDef rtcDate = DateToRtcDate(GetTime(epochTime));
    RTC_TimeTypeDef rtcTime = TimeToRtcTime(GetTime(epochTime));

    // Set Date in RTC
    if(HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN)  != HAL_OK)
    {
        return false;
    }

    // Set Time in RTC
    if(HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN)  != HAL_OK)
    {
        return false;
    }

    return true;
}

//------------------------------------------------------------------
// SetTime
//------------------------------------------------------------------
bool Clock::SetTime(tm time)
{
    return SetTime(mktime(&time));
}

//------------------------------------------------------------------
// GetTime
//------------------------------------------------------------------
tm Clock::GetTime(time_t eTime)
{
    tm* tempTime = NULL;
    tempTime = gmtime(&eTime);
    return *tempTime;
}

//------------------------------------------------------------------
// GetTime
//------------------------------------------------------------------
time_t Clock::GetCurrentTimeEpoch()
{
    UpdateTime();
    return epochTime;
}

//------------------------------------------------------------------
// UpdateTime
//------------------------------------------------------------------
void Clock::UpdateTime()
{
    RTC_DateTypeDef rtcDate;
    RTC_TimeTypeDef rtcTime;

    // Get RTC Date and time
    rtcTime = GetRtcTime();
    rtcDate = GetRtcDate();

    // check for errors
    if (0 == rtcDate.Year || 2 == rtcTime.TimeFormat)
    {
        ClearTime();
    }

    // convert to epoch time
    epochTime = RtcTimeToEpochTime(rtcDate, rtcTime);
}

//------------------------------------------------------------------
// GetRtcDate
//------------------------------------------------------------------
RTC_DateTypeDef Clock::GetRtcDate()
{
    RTC_DateTypeDef rtcDate = {0};

    // check for error while reading date from RTC
    if (HAL_RTC_GetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN) != HAL_OK)
    {
        rtcDate.Year = 0;
    }

    return rtcDate;
}

//------------------------------------------------------------------
// GetRtcTime
//------------------------------------------------------------------
RTC_TimeTypeDef Clock::GetRtcTime()
{
    RTC_TimeTypeDef rtcTime = {0};

    // check for error while reading time from RTC
    if (HAL_RTC_GetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN) != HAL_OK)
    {
        rtcTime.TimeFormat = 2;
    }

    return rtcTime;
}

//------------------------------------------------------------------
// RtcTimeToEpochTime
//------------------------------------------------------------------
time_t Clock::RtcTimeToEpochTime(RTC_DateTypeDef rtcDate, RTC_TimeTypeDef rtcTime)
{
    tm time;
    time_t eTime = 0;

    // convert RTC Date to struct tm date
    time.tm_year = static_cast<int>(rtcDate.Year) + 100; // time.h is years since 1900, chip is years since 2000
    time.tm_mday = static_cast<int>(rtcDate.Date);
    time.tm_wday = static_cast<int>(rtcDate.WeekDay);
    time.tm_mon = static_cast<int>(rtcDate.Month) - 1; // time.h is months since January, 0-11; chip is 1-12

    // convert RTC Time to struct tm time
    time.tm_hour = static_cast<int>(rtcTime.Hours);
    time.tm_min = static_cast<int>(rtcTime.Minutes);
    time.tm_sec = static_cast<int>(rtcTime.Seconds);

    eTime = mktime(&time);

    return eTime;
}

//------------------------------------------------------------------
// DateToRtcDate
//------------------------------------------------------------------
RTC_DateTypeDef Clock::DateToRtcDate(tm time)
{
    RTC_DateTypeDef rtcDate = {0};

    // convert struct tm date to RTC date format
    rtcDate.Year = static_cast<uint8_t>(time.tm_year - 100); // time.h is years since 1900, chip is years since 2000
    rtcDate.Date = static_cast<uint8_t>(time.tm_mday);
    rtcDate.WeekDay = static_cast<uint8_t>(time.tm_wday); // time.h is days since Sunday, 0-6 Sunday - Saturday; chip is 1-7, Monday - Sunday
    if (0 == rtcDate.WeekDay) // 0 = Sunday from time.h, 7 = Sunday for RTC
    {
        rtcDate.WeekDay = 7;
    }
    rtcDate.Month = static_cast<uint8_t>(time.tm_mon + 1); // time.h is months since January, 0-11; chip is 1-12

    return rtcDate;
}

//------------------------------------------------------------------
// TimeToRtcTime
//------------------------------------------------------------------
RTC_TimeTypeDef Clock::TimeToRtcTime(tm time)
{
    RTC_TimeTypeDef rtcTime = {0};

    // convert struct tm time to RTC time format
    rtcTime.Hours = static_cast<uint8_t>(time.tm_hour);
    rtcTime.Minutes = static_cast<uint8_t>(time.tm_min);
    rtcTime.Seconds = static_cast<uint8_t>(time.tm_sec);

    return rtcTime;
}

//------------------------------------------------------------------
// ClearTime
//------------------------------------------------------------------
void Clock::ClearTime()
{
    epochTime = 0;
}
