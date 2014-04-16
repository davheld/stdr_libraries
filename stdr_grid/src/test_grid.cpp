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


#include <stdr_grid/grid.h>
#include <stdr_grid/rolling_grid.h>
#include <gtest/gtest.h>

using namespace stdr;

struct mycell
{
  int i;
  mycell() : i(0) { }
};

TEST(Grid, AllocationBuiltinTypeNoDefault)
{
  Grid<int> grid(.1,5,5);
  std::cout <<"Default int value: " << *grid[0] <<std::endl;
}

TEST(Grid, AllocationBuiltinTypeWithDefault)
{
  Grid<int> grid(.1,5,5,0);
  EXPECT_EQ(0, *grid[0]);
}

TEST(Grid, AllocationStructNoDefault)
{
  Grid<mycell> grid(.1,5,5);
  EXPECT_EQ(0, grid[0]->i);
}

TEST(Grid, AllocationStructWithDefault)
{
  mycell default_val;
  default_val.i = 1;
  Grid<mycell> grid(.1,5,5,default_val);
  EXPECT_EQ(1, grid[0]->i);
}

TEST(Grid, SetDimension)
{
  Grid<mycell> grid;
  grid.setDimensions(.1,5,5);
  EXPECT_EQ(0, grid[0]->i);
}



TEST(RollingGrid, AllocationBuiltinTypeWithDefault)
{
  RollingGrid<int> grid(.1,5,5,0);
  EXPECT_EQ(0, *grid[0]);
}

TEST(RollingGrid, AllocationStructNoDefault)
{
  RollingGrid<mycell> grid(.1,5,5);
  EXPECT_EQ(0, grid[0]->i);
}

TEST(RollingGrid, AllocationStructWithDefault)
{
  mycell default_val;
  default_val.i = 1;
  RollingGrid<mycell> grid(.1,5,5,default_val);
  EXPECT_EQ(1, grid[0]->i);
}

TEST(RollingGrid, SetDimension)
{
  RollingGrid<mycell> grid;
  grid.setDimensions(.1,5,5);
  EXPECT_EQ(0, grid[0]->i);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
