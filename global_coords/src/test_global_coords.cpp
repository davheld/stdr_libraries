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

#include <cmath>
#include <gtest/gtest.h>
#include <global_coords/global_coords.h>

using namespace global_coords;

#define __TOLERANCE__ 0.001

#define COMPARE_UTM(u0, u1) do { \
  EXPECT_LT( ::fabs(u0.x-u1.x), __TOLERANCE__ ); \
  EXPECT_LT( ::fabs(u0.y-u1.y), __TOLERANCE__ ); \
  EXPECT_EQ(u0.zone, u1.zone); \
  } while(0)

#define COMPARE_LL(ll0, ll1) do { \
  EXPECT_LT( ::fabs(ll0.lat-ll1.lat), __TOLERANCE__ ); \
  EXPECT_LT( ::fabs(ll0.lon-ll1.lon), __TOLERANCE__ ); \
  } while(0)


TEST(GlobalCoords, Conversion)
{
  // start from lat lon
  LatLonCoords ll0(37.430421, -122.184260);

  // convert to utm a first time
  UtmCoords u0 = latLonToUtm(ll0);

  // convert back to lat lon
  LatLonCoords ll1 = utmToLatLon(u0);

  // and compare with the original value
  COMPARE_LL(ll1, ll0);

  // convert again to utm
  UtmCoords u1 = latLonToUtm(ll1);

  // and compare with the value we obtained the first time
  COMPARE_UTM(u1, u0);
}

TEST(GlobalCoords, CrossAssignments)
{
  const LatLonCoords ll0(37.430421, -122.184260);
  const UtmCoords u0 = latLonToUtm(ll0);

  UtmCoords u = ll0;
  COMPARE_UTM(u, u0);

  LatLonCoords ll = u0;
  COMPARE_LL(ll, ll0);
}

#define CHECK_DUAL(d) COMPARE_LL(d.ll(), ll0); COMPARE_UTM(d.utm(), u0)

TEST(GlobalCoords, DualCoordsAssignments)
{
  const LatLonCoords ll0(37.430421, -122.184260);
  const UtmCoords u0 = latLonToUtm(ll0);

  DualCoords dc0;
  DualCoords dc1(dc0);
  DualCoords dc2; dc2 = dc1;

  {
    DualCoords dc;
    dc.set(ll0);
    CHECK_DUAL(dc);
    dc.set(u0);
    CHECK_DUAL(dc);
  }

  {
    DualCoords dc( u0 );
    CHECK_DUAL(dc);
  }

  {
    DualCoords dc( ll0 );
    CHECK_DUAL(dc);
  }

  dc0 = u0;
  CHECK_DUAL(dc0);
  dc0 = ll0;
  CHECK_DUAL(dc0);
}


#define COMPARE_DUAL(d0, d1) COMPARE_LL(d0.ll(), d1.ll()); COMPARE_UTM(d0.utm(), d1.utm())
#define COMPARE_VEC(v0, v1) EXPECT_LT( ((v1)-(v0)).norm(), __TOLERANCE__ )

TEST(GlobalCoords, Operators)
{
  const LatLonCoords ll0(37.430421, -122.184260);
  const UtmCoords u0 = ll0;
  const DualCoords d0 = ll0;
  const Eigen::Vector2d v(100, 100);
  const UtmCoords u1(u0.x + v(0), u0.y + v(1), u0.zone);
  const LatLonCoords ll1 = u1;
  const DualCoords d1 = u1;

  LatLonCoords ll;
  UtmCoords u;
  DualCoords d;

  ll = ll0; ll += v;
  COMPARE_LL(ll, ll1);

  ll = ll0 + v;
  COMPARE_LL(ll, ll1);

  u = u0; u += v;
  COMPARE_UTM(u, u1);

  u = u0 + v;
  COMPARE_UTM(u, u1);

  d = d0; d += v;
  COMPARE_DUAL(d, d1);

  d = d0 + v;
  COMPARE_DUAL(d, d1);

  COMPARE_VEC(ll1-ll0, v);
  COMPARE_VEC(u1-u0, v);
  COMPARE_VEC(d1-d0, v);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
