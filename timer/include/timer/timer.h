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


#ifndef HIGH_RES_TIMER_H
#define HIGH_RES_TIMER_H

#include <time.h>
#include <string>
#include <sstream>
#include <cstddef>
#include <iostream>
#include <cstdio>

/// /brief Time units to control reporting
struct TimeUnit
{
  enum Unit {
    AUTO,
    US,
    MS,
    SEC,
    MIN,
    HR
  };
};

//! CLOCK_MONOTONIC_RAW will not be adjusted by NTP.
//! See man clock_gettime.
class HighResTimer
{
public:
  std::string description_;

  HighResTimer(const std::string& description = "HighResTimer");
  void start();
  void stop();
  void reset(const std::string& description);
  void reset();
  double getMicroseconds() const;
  double getMilliseconds() const;
  double getSeconds() const;
  double getMinutes() const;
  double getHours() const;

  std::string report() const;
  std::string report(TimeUnit::Unit unit) const;
  std::string reportMicroseconds() const;
  std::string reportMilliseconds() const;
  std::string reportSeconds() const;
  std::string reportMinutes() const;
  std::string reportHours() const;

  void print() const {std::string msString = report(); printf("[TIMER] %s\n", msString.c_str());}
  void printSeconds() const {std::string msString = reportSeconds(); printf("[TIMER] %s\n", msString.c_str());}
  void printMilliseconds() const {std::string msString = reportMilliseconds(); printf("[TIMER] %s\n", msString.c_str());}
  void printMicroseconds() const {std::string msString = reportMicroseconds(); printf("[TIMER] %s\n", msString.c_str());}

private:
  double total_us_;
  timespec start_;
  timespec end_;
  bool stopped_;
};

class ScopedTimer
{
public:
  HighResTimer hrt_;
  ScopedTimer(const std::string& description = "ScopedTimer", TimeUnit::Unit unit=TimeUnit::AUTO);
  ~ScopedTimer();

private:
  TimeUnit::Unit unit_;
};

#endif // HIGH_RES_TIMER_H
