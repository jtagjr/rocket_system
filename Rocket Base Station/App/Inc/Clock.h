
#ifndef CLOCK_HPP_
#define CLOCK_HPP_

#include <time.h>
#include "stm32f4xx_hal.h"

/// @brief Clock class
/// @details This class serves as an interface to the RTC and handles conversions from time.h formats to RTC formats. The intent
///          in to set the RTC with the latest epoch UTC time and then use the RTC to keep time while the device is operating on battery.
class Clock
{
    public:

        /// @brief Only need one instance of the clock
        /// @return instance to this object
        static Clock& GetInstance();

        /// @brief Set time in the RTC with given UTC epoch time
        /// @param eTime - time in epoch format (an integral value representing the number of seconds elapsed since 00:00 hours, Jan 1, 1970 UTC)
        /// @return true if successful, false otherwise
        bool SetTime(time_t eTime);

        /// @brief Set time in the RTC with given time.h struct tm format
        /// @param time - time in time.h struct tm format
        /// @see time.h
        /// @return true if successful, false otherwise
        bool SetTime(tm time);

        /// @brief convert epoch time to time.h tm struct
        /// @param eTime - time in epoch format
        /// @return current time from RTC (time.h struct tm format)
        tm GetTime(time_t eTime);

        /// @brief Get the current time, from RTC, in epoch format
        /// @return current time from RTC (epoch format)
        time_t GetCurrentTimeEpoch();

        Clock(const Clock&) = delete;
        Clock& operator=(const Clock&) = delete;

    private:

        /// @brief Constructor
        Clock();

        /// @brief Update this objects time values based on current RTC time
        void UpdateTime();

        /// @brief Get current RTC Date
        /// @return Date in RTC format
        RTC_DateTypeDef GetRtcDate();

        /// @brief Get current RTC Time
        /// @return Time in RTC format
        RTC_TimeTypeDef GetRtcTime();

        /// @brief Convert time.h struct tm format to RTC format for Date
        /// @param time - time structure
        /// @return Date in RTC format
        RTC_DateTypeDef DateToRtcDate(tm time);

        /// @brief Convert time.h struct tm format to RTC format for Time
        /// @param time - time structure
        /// @return Time in RTC format
        RTC_TimeTypeDef TimeToRtcTime(tm time);

        /// @brief Convert RTC Time and RTC Date to time.h struct tm format
        /// @param rtcDate - Date in RTC format
        /// @param rtcTime - Time in RTC format
        /// @return time in epoch format
        time_t RtcTimeToEpochTime(RTC_DateTypeDef rtcDate, RTC_TimeTypeDef rtcTime);

        /// @brief Clear object time values (all 0's)
        void ClearTime();

        time_t epochTime;
};

#endif // CLOCK_HPP_
