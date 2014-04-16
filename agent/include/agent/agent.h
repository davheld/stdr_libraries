#ifndef AGENT_H
#define AGENT_H

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <agent/lockable.h>

typedef boost::shared_ptr<boost::thread> ThreadPtr;

//! All of this is highly experimental.
class Agent : public SharedLockable
{
public:
  Agent() : SharedLockable(), quitting_(false), running_(false) {}
  virtual ~Agent() {}

  //! TODO: should probably be 'stop'.
  void quit() { scopeLockWrite; quitting_ = true; }
  bool running() { scopeLockRead; return running_; }
  void run() { running_ = true; _run(); running_ = false; }
  virtual void _run() = 0;
  //! This should be a void function.
  ThreadPtr launch()
  {
    thread_ = ThreadPtr(new boost::thread(boost::bind(&Agent::run, this)));
    return thread_;
  }
  ThreadPtr thread() const { return thread_; }

  //! This should maybe not exist at all.
  void detach() { boost::thread thread(boost::bind(&Agent::run, this)); thread.detach(); }
  
protected:
  ThreadPtr thread_;
  bool quitting_;
  bool running_;
};

#endif // AGENT_H
