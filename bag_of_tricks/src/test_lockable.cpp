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
#include <gtest/gtest.h>

using namespace std;

class Foo : public SharedLockable
{
public:
  Foo() : SharedLockable() {}
  void deadlock() { scopeLockWrite; lockWrite(); }
  int val() { scopeLockRead; return val_; }
  void setVal(int val) { scopeLockWrite; val_ = val; }
  
protected:
  int val_;
};

// TEST(Lockable, foo)
// {
//   Foo foo;
//   cout << "Trying to deadlock..." << endl;
//   foo.deadlock();
//   cout << "Did not deadlock." << endl;
//   EXPECT_TRUE(false);
// }

TEST(Lockable, Copy)
{
  Foo foo;
  foo.setVal(37);
  foo.lockWrite();
  Foo bar = foo;
  bar.lockWrite();  // This should not deadlock; copying the SharedLockable does *not* copy the lock state.
  Foo baz;
  baz.setVal(12);
  baz = bar;

  EXPECT_TRUE(baz.val() == 37);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
