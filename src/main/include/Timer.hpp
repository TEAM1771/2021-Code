/**
 * This file includes a lot of code copied and modified from frc::Timer
 * The only difference is that it returns std::chrono utilities rather than frc ones
 */

#pragma once

#include <chrono>
#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <thread>
#include <wpi/mutex.h>

namespace ngr
{
    /// mimmics frc::GetTime() but returns a std::chrono::time_point
    inline auto GetTime()
    {
        return std::chrono::system_clock::now().time_since_epoch();
    }

    /**
     * This is designed to mimic frc::Timer but unlike frc::Timer, this will 
     * not convert out of chrono types
     * 
     * It also has a less stupid implimentation of HasPeriodPassed(period)
     */
    class Timer
    {
    public:
        /**
         * Create a new timer object.
         *
         * Create a new timer object and reset the time to zero. The timer is
         * initially not running and must be started.
         */
        Timer();

        virtual ~Timer() = default;

        Timer(const Timer& rhs);
        Timer& operator=(const Timer& rhs);
        Timer(Timer&& rhs);
        Timer& operator=(Timer&& rhs);

        /**
         * Get the current time from the timer. If the clock is running it is derived
         * from the current system clock the start time stored in the timer class. If
         * the clock is not running, then return the time when it was last stopped.
         *
         * @return Current time value for this timer in seconds
         */
        auto Get() const
        {
            auto const currentTime = GetFPGATimestamp();

            std::scoped_lock lock(m_mutex);
            if(m_running)
                return (currentTime - m_startTime) + m_accumulatedTime;
            else
                return m_accumulatedTime;
        }

        /**
         * Reset the timer by setting the time to 0.
         *
         * Make the timer startTime the current time so new requests will be relative
         * to now.
         */
        void Reset();

        /**
         * Start the timer running.
         *
         * Just set the running flag to true indicating that all time requests should
         * be relative to the system clock.
         */
        void Start();

        /**
         * Stop the timer.
         *
         * This computes the time as of now and clears the running flag, causing all
         * subsequent time requests to be read from the accumulated time rather than
         * looking at the system clock.
         */
        void Stop();

        /**
         * Check if the period specified has passed and if it has, advance the start
         * time by that period. This is useful to decide if it's time to do periodic
         * work without drifting later by the time it took to get around to checking.
         *
         * @param period                The period to check for.
         * @param do_default_behaviour  If true, this advances the start time as stated above
         * @return                      True if the period has passed.
         */
        template <typename REP, typename RATIO>
        bool HasPeriodPassed(std::chrono::duration<REP, RATIO> period, bool do_default_behaviour = false)
        {
            if(Get() > period)
            {
                std::scoped_lock lock(m_mutex);
                // Advance the start time by the period.
                if(do_default_behaviour)
                    m_startTime += period;
                // Don't set it to the current time... we want to avoid drift.
                return true;
            }
            return false;
        }

        /**
         * Return the FPGA system clock time in seconds.
         *
         * Return the time from the FPGA hardware clock in seconds since the FPGA
         * started. Rolls over after 71 minutes.
         *
         * @returns Robot running time in seconds.
         */
        static std::chrono::microseconds GetFPGATimestamp();

        /**
         * Return the approximate match time.
         *
         * The FMS does not send an official match time to the robots, but does send
         * an approximate match time. The value will count down the time remaining in
         * the current period (auto or teleop).
         *
         * Warning: This is not an official time (so it cannot be used to dispute ref
         * calls or guarantee that a function will trigger before the match ends).
         *
         * The Practice Match function of the DS approximates the behavior seen on the
         * field.
         *
         * @return Time remaining in current match period (auto or teleop)
         */
        static std::chrono::duration<double> GetMatchTime();

    private:
        std::chrono::microseconds m_startTime = GetFPGATimestamp();
        std::chrono::microseconds m_accumulatedTime { 0 };
        bool                      m_running = false;
        mutable wpi::mutex        m_mutex;
    };
} // namespace ngr