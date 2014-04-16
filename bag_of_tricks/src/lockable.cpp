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

#include <agent/lockable.h>

namespace bag_of_tricks
{

Lockable::Lockable() :
  mutex_(pthread_mutex_t())
{
}

void Lockable::lock()
{
  pthread_mutex_lock(&mutex_);
}

void Lockable::unlock()
{
  pthread_mutex_unlock(&mutex_);
}

bool Lockable::trylock()
{
  if(pthread_mutex_trylock(&mutex_) == EBUSY)
    return false;
  else
    return true;
}


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

} //namespace bag_of_tricks
