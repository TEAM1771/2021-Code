#include "Timer.hpp"
#include "frc/DriverStation.h"
#include "frc/RobotController.h"
#include <chrono>
#include <hal/FRCUsageReporting.h>
#include <thread>

using namespace std::literals::chrono_literals;
using namespace ngr;

Timer::Timer()
{
    Reset();
}

Timer::Timer(const Timer& rhs)
    : m_startTime(rhs.m_startTime)
    , m_accumulatedTime(rhs.m_accumulatedTime)
    , m_running(rhs.m_running)
{}

Timer& Timer::operator=(const Timer& rhs)
{
    std::scoped_lock lock(m_mutex, rhs.m_mutex);

    m_startTime       = rhs.m_startTime;
    m_accumulatedTime = rhs.m_accumulatedTime;
    m_running         = rhs.m_running;

    return *this;
}

Timer::Timer(Timer&& rhs)
    : m_startTime(std::move(rhs.m_startTime))
    , m_accumulatedTime(std::move(rhs.m_accumulatedTime))
    , m_running(std::move(rhs.m_running))
{}

Timer& Timer::operator=(Timer&& rhs)
{
    std::scoped_lock lock(m_mutex, rhs.m_mutex);

    m_startTime       = std::move(rhs.m_startTime);
    m_accumulatedTime = std::move(rhs.m_accumulatedTime);
    m_running         = std::move(rhs.m_running);

    return *this;
}

void Timer::Reset()
{
    std::scoped_lock lock(m_mutex);
    m_accumulatedTime = 0s;
    m_startTime       = GetFPGATimestamp();
}

void Timer::Start()
{
    std::scoped_lock lock(m_mutex);
    if(! m_running)
    {
        m_startTime = GetFPGATimestamp();
        m_running   = true;
    }
}

void Timer::Stop()
{
    auto const temp = Get();

    std::scoped_lock lock(m_mutex);
    if(m_running)
    {
        m_accumulatedTime = temp;
        m_running         = false;
    }
}


std::chrono::microseconds Timer::GetFPGATimestamp()
{
    // FPGA returns the timestamp in microseconds
    return std::chrono::microseconds(frc::RobotController::GetFPGATime());
}

std::chrono::duration<double> Timer::GetMatchTime()
{
    return std::chrono::duration<double, std::ratio<1, 1>>(frc::DriverStation::GetInstance().GetMatchTime());
}