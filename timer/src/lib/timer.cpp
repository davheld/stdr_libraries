#include <ros/ros.h>
#include <timer/timer.h>

#if defined(__MACH__) && defined(__APPLE__)
#include <mach/mach_time.h>
void get_time(timespec * t) {
  // TODO
  uint64_t tv = mach_absolute_time();
  static mach_timebase_info_data_t info;
  if(info.denom == 0) {
    kern_return_t ret = mach_timebase_info(&info);
  }

  uint64_t nanosec = tv * info.numer / info.denom;
  t->tv_sec  = nanosec / 1000000000;
  t->tv_nsec = nanosec % 1000000000;
  return;
}
#else
#define HRTCLOCK CLOCK_MONOTONIC_RAW
void get_time(timespec * t) {
  clock_gettime(HRTCLOCK, t);
}
#endif


HighResTimer::HighResTimer(const std::string& description) :
  description_(description),
  total_us_(0),
  stopped_(true)
{
}

void HighResTimer::start()
{
  get_time( &start_);
  stopped_ = false;
}

void HighResTimer::stop()
{
  get_time( &end_);
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
    get_time( &curr);
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


