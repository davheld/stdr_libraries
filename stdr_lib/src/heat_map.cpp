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
#include <algorithm>
#include <stdr_lib/heat_map.h>


namespace stdr
{


// adapted from http://www.cs.rit.edu/~ncs/color/t_convert.html
void hsv2rgb(float h, float s, float v,
             unsigned char *r, unsigned char *g, unsigned char *b)
{
  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v * 255;
    return;
  }

  h /= 60;
  const int i = floor(h); // sector 0 to 5
  const float f = h - i;  // factorial part of h
  const float p = v * ( 1 - s );
  const float q = v * ( 1 - s * f );
  const float t = v * ( 1 - s * ( 1 - f ) );

  float _r, _g, _b;

  switch( i ) {
    case 0:
      _r = v;
      _g = t;
      _b = p;
      break;
    case 1:
      _r = q;
      _g = v;
      _b = p;
      break;
    case 2:
      _r = p;
      _g = v;
      _b = t;
      break;
    case 3:
      _r = p;
      _g = q;
      _b = v;
      break;
    case 4:
      _r = t;
      _g = p;
      _b = v;
      break;
    default:    // case 5:
      _r = v;
      _g = p;
      _b = q;
      break;
  }
  *r = _r * 255;
  *g = _g * 255;
  *b = _b * 255;
}

void heatmap(float val, float min, float max,
             unsigned char *r, unsigned char *g, unsigned char *b)
{
  val = (val-min)/(max-min);
  if( val>1 ) val=1;
  if( val<0 ) val=0;

  // In HSV color space, h=0 is red, h=240 is deep blue.
  // With the following formula, we have red for max values of val, and blue
  // for low values.
  const float h = 240 * (1-val);

  hsv2rgb(h, 1, 1, r, g, b);
}

ColorWheel::ColorWheel()
{

}

ColorWheel::ColorWheel(unsigned n)
  : rgbs_(n)
{
  for(unsigned i=0; i<n; ++i)
    heatmap(((float)i)/n, 0, 1, &rgbs_[i].r, &rgbs_[i].g, &rgbs_[i].b);
  std::random_shuffle(rgbs_.begin(), rgbs_.end());
}

//          0       1     2         3             4         5            6               7        8       9
// Colors: green, cyan, yellow, light orange, dark orange,  green?      light blue            green     cyan
/// Returns the n-th color (modulo the number of colors)
const ColorWheel::RGB& ColorWheel::operator[] (unsigned n) const {
  int color = n % rgbs_.size();
  if (n == 178 || n == 168 || n == 51) {
    color = 3; // orange
  } else if (n == 132) {
    color = 2; // yellow
  }

  return rgbs_[color];
}

} // namespace stdr
