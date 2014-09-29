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

#ifndef __STDR_LIB__HEAT_MAP__H__
#define __STDR_LIB__HEAT_MAP__H__


#include <vector>

namespace stdr
{

/** Converts from HSV to RGB.
  *
  * @param h hue from 0 to 360
  * @param s saturation from 0 to 1
  * @param v value from 0 to 1
  * @param r red component from 0 to 255
  * @param g green component from 0 to 255
  * @param b blue component from 0 to 255
  */
void hsv2rgb(float h, float s, float v,
             unsigned char *r, unsigned char *g, unsigned char *b);

/** Computes RGB values corresponding to the value of val in our heatmap.
 *
 * large values will turn as red, and low values as blue. Values will be
 * bounded between min and max.
 * The resulting RGB will be between 0 and 255.
 */
void heatmap(float val, float min, float max,
             unsigned char *r, unsigned char *g, unsigned char *b);


/** A class to hold a number of colors drawn at random from the heat map */
class ColorWheel
{
public:
  struct RGB { unsigned char r, g, b; };

  /// Constructs an empty color wheel
  ColorWheel();

  /// Constructs a color wheel with n colors from red to blue
  explicit ColorWheel(unsigned n);

  inline bool empty() const { return rgbs_.empty(); }
  inline unsigned size() const { return rgbs_.size(); }

  /// Returns the n-th color (modulo the number of colors)
  inline const RGB& operator[] (unsigned n) const { return rgbs_[n%rgbs_.size()]; }

private:
  std::vector<RGB> rgbs_;
};


} // namespace stdr


#endif // __STDR_LIB__HEAT_MAP__H__
