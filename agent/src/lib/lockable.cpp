#include <agent/lockable.h>

SharedLockable::SharedLockable()
{
}

void SharedLockable::lockWrite()
{
  shared_mutex_.lock();
}

void SharedLockable::unlockWrite()
{
  shared_mutex_.unlock();
}

bool SharedLockable::trylockWrite()
{
  return shared_mutex_.try_lock();
}

void SharedLockable::lockRead()
{
  shared_mutex_.lock_shared();
}

void SharedLockable::unlockRead()
{
  shared_mutex_.unlock_shared();
}

bool SharedLockable::trylockRead()
{
  return shared_mutex_.try_lock_shared();
}

