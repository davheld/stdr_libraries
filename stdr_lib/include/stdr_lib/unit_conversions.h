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

#ifndef __STDR_COMMON__UNIT_CONVERSIONS_H__
#define __STDR_COMMON__UNIT_CONVERSIONS_H__

namespace stdr
{

inline double mph2ms(double mph)
{ return (mph * 0.44704); }

inline double ms2mph(double ms)
{ return (ms * 2.23693629); }

inline double kph2ms(double kph)
{ return (kph * 0.277777778); }

inline double ms2kph(double ms)
{ return (ms * 3.6); }

inline double meters2feet(double meters)
{ return (meters * 3.2808399); }

inline double feet2meters(double feet)
{ return (feet * 0.3048); }

inline double meters2miles(double meters)
{  return (meters / 1609.344); }

inline double miles2meters(double miles)
{ return (miles * 1609.344); }

inline double mph2kph(double mph)
{ return (mph * 1.609344); }

inline double kph2mph(double kph)
{ return (kph * 0.621371192); }

} //namespace stdr

#endif // __STDR_COMMON__UNIT_CONVERSIONS_H__
