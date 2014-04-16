/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that the
  following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include <ros/ros.h>
#include <timer/timer.h>

#define HRTCLOCK CLOCK_MONOTONIC_RAW


namespace bag_of_tricks
{

HighResTimer::HighResTimer(const std::string& description) :
  description_(description),
  total_us_(0),
  stopped_(true)
{
}

void HighResTimer::start()
{
  clock_gettime(HRTCLOCK, &start_);
  stopped_ = false;
}

void HighResTimer::stop()
{
  clock_gettime(HRTCLOCK, &end_);
  total_us_ += 1e6 * (end_.tv_sec - start_.tv_sec) + 1e-3 * (end_.tv_nsec - start_.tv_nsec);
  stopped_ = true;
}

void HighResTimer::reset(const std::string& description)
{
  description_ = description;
  total_us_ = 0;
  stopped_ = true;
}

void HighResTimer::reset()
{
  total_us_ = 0;
  stopped_ = true;
}

double HighResTimer::getMicroseconds() const
{
  if(stopped_)
    return total_us_;
  else { 
    timespec curr;
    clock_gettime(HRTCLOCK, &curr);
    return total_us_ + 1e6 * (curr.tv_sec - start_.tv_sec) + 1e-3 * (curr.tv_nsec - start_.tv_nsec);
  }
}

double HighResTimer::getMilliseconds() const
{
  return getMicroseconds() / 1000.;
}

double HighResTimer::getSeconds() const
{
  return getMilliseconds() / 1000.;
}

double HighResTimer::getMinutes() const
{
  return getSeconds() / 60.;
}

double HighResTimer::getHours() const
{
  return getMinutes() / 60.;
}

std::string HighResTimer::reportMicroseconds() const
{
  std::ostringstream oss; oss << description_ << ": " << getMicroseconds() << " microseconds.";
  return oss.str();
}

std::string HighResTimer::reportMilliseconds() const
{
  std::ostringstream oss; oss << description_ << ": " << getMilliseconds() << " milliseconds.";
  return oss.str();
}

std::string HighResTimer::reportSeconds() const
{
  std::ostringstream oss; oss << description_ << ": " << getSeconds() << " seconds.";
  return oss.str();
}

std::string HighResTimer::reportMinutes() const
{
  std::ostringstream oss; oss << description_ << ": " << getMinutes() << " minutes.";
  return oss.str();
}

std::string HighResTimer::reportHours() const
{
  std::ostringstream oss; oss << description_ << ": " << getHours() << " hours.";
  return oss.str();
}

std::string HighResTimer::report() const
{
  double val = getMicroseconds();
  if(val <= 1000.0)
    return reportMicroseconds();

  val /= 1000.0;
  if(val <= 1000.0 && val >= 1.0)
    return reportMilliseconds();

  val /= 1000.0;
  if(val <= 60.0 && val >= 1.0)
    return reportSeconds();
  
  val /= 60.0;
  if(val <= 60.0 && val >= 1.0)
    return reportMinutes();
  
  val /= 60.0;
  return reportHours();
}

std::string HighResTimer::report(TimeUnit::Unit unit) const
{
  switch(unit)
  {
  case TimeUnit::AUTO: return report();
  case TimeUnit::US: return reportMicroseconds();
  case TimeUnit::MS: return reportMilliseconds();
  case TimeUnit::SEC: return reportSeconds();
  case TimeUnit::MIN: return reportMinutes();
  case TimeUnit::HR: return reportHours();
  }
  ROS_BREAK();
  return "";
}

ScopedTimer::ScopedTimer(const std::string& description, TimeUnit::Unit unit)
: hrt_(description)
, unit_(unit)
{
  hrt_.start();
}

ScopedTimer::~ScopedTimer()
{
  hrt_.stop();
  std::cout << hrt_.report(unit_) << std::endl;
}


} //namespace bag_of_tricks


