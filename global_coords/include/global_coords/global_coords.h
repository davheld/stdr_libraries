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

#ifndef __GLOBAL_COORDS_H__
#define __GLOBAL_COORDS_H__

#include <string>

#include <Eigen/Core>


/** @file

    @brief Defines data structures and an API to work with global coordinates.

    We are currently working with UTM and LatLon coordinates.

    @todo In the future we want to get rid of UTM coordinates, and use a local
    cartesian space instead (see GeographicLib::LocalCartesian).

    @author Brice Rebsamen
  */


namespace global_coords {


// forward declaration
struct UtmCoords;

/// @brief A structure to hold global coordinates as latitude and longitude
struct LatLonCoords
{
  double lat; ///< Latitude
  double lon; ///< Longitude

  /// Default constructor. Initializes to 0.
  LatLonCoords() : lat(.0), lon(.0) { }

  /// Constructs from given latitude and longitude.
  LatLonCoords(double _lat, double _lon) : lat(_lat), lon(_lon) { }

  /// Constructs from utm coordinates (calls latLonToUtm()).
  LatLonCoords(const UtmCoords&);

  /// Assignment operator from utm coordinates (calls latLonToUtm()).
  LatLonCoords& operator= (const UtmCoords&);


  /// @{ \name arithmetic operators

  /// Adds a displacement given in euclidean space.
  LatLonCoords& operator+= (const Eigen::Vector2d&);

  /// Substracts a displacement given in euclidean space.
  LatLonCoords& operator-= (const Eigen::Vector2d&);

  /// Adds a displacement given in euclidean space.
  const LatLonCoords operator+ (const Eigen::Vector2d& other) const {
    return LatLonCoords(*this) += other;
  }

  /// Substracts a displacement given in euclidean space.
  const LatLonCoords operator- (const Eigen::Vector2d& other) const {
    return LatLonCoords(*this) -= other;
  }

  /// @}
};


/// @brief A structure to hold global coordinates as UTM coordinates.
struct UtmCoords
{
  double x; ///< x, or easting
  double y; ///< y, or northing
  std::string zone; ///< utm zone

  /// Default constructor. Initializes to 0.
  UtmCoords() : x(.0), y(.0) { }

  /// Constructs from x, y, zone.
  UtmCoords(double _x, double _y, const std::string & _z) : x(_x), y(_y), zone(_z) { }

  /// Constructs from latitude and longitude (calls utmToLatLon()).
  UtmCoords(const LatLonCoords&);

  /// Assignment operator from latitude and longitude (calls utmToLatLon()).
  UtmCoords& operator= (const LatLonCoords&);


  /// @{ \name arithmetic operators

  /// Adds a displacement given in euclidean space.
  UtmCoords& operator+= (const Eigen::Vector2d&);

  /// Substracts a displacement given in euclidean space.
  UtmCoords& operator-= (const Eigen::Vector2d&);

  /// Adds a displacement given in euclidean space.
  const UtmCoords operator+ (const Eigen::Vector2d& other) const {
    return UtmCoords(*this) += other;
  }

  /// Substracts a displacement given in euclidean space.
  const UtmCoords operator- (const Eigen::Vector2d& other) const {
    return UtmCoords(*this) -= other;
  }

  /// @}
};


/// Converts from LatLonCoords to UtmCoords.
UtmCoords latLonToUtm(const LatLonCoords &);

/// Converts latitude and longitude to UtmCoords.
inline UtmCoords latLonToUtm(double lat, double lon)
{ return latLonToUtm( LatLonCoords(lat, lon) ); }

/// Converts from UtmCoords to LatLonCoords.
LatLonCoords utmToLatLon(const UtmCoords &);

/// Converts from utm coordinates to LatLonCoords.
inline LatLonCoords utmToLatLon(double x, double y, const std::string& z)
{ return utmToLatLon( UtmCoords(x,y,z) ); }



/** @brief A class to hold the position of a point both as LatLonCoords and UtmCoords.

    This maintains a dual representation of the point. Updating one also
    updates the other.
  */
class DualCoords
{
private:
  LatLonCoords ll_; ///< LatLon representation
  UtmCoords utm_; ///< UTM representation

public:
  /// Default constructor. Initializes to lat=lon=0, and utm from latlon.
  DualCoords() : utm_(ll_) { }

  /// Constructs from LatLonCoords (and sets the utm representation accordingly).
  DualCoords(const LatLonCoords& ll) { set(ll); }

  /// Constructs from UtmCoords (and sets the latlon representation accordingly).
  DualCoords(const UtmCoords& u) { set(u); }

  /// Assignment operator from LatLonCoords (sets the utm representation accordingly).
  DualCoords& operator= (const LatLonCoords& ll) { set(ll); return *this; }

  /// Assignment operator from UtmCoords (sets the latlon representation accordingly).
  DualCoords& operator= (const UtmCoords& u) { set(u); return *this; }

  /// Sets the coordinates from LatLonCoords (and sets the utm representation accordingly).
  void set(const LatLonCoords& ll) { ll_=ll; utm_=latLonToUtm(ll_); }

  /// Sets the coordinates from UtmCoords (and sets the latlon representation accordingly).
  void set(const UtmCoords& u) { utm_=u; ll_=utmToLatLon(utm_); }

  /// Sets the coordinates from lat and lon (and sets the utm representation accordingly).
  void setLatLon(double lat, double lon) { set(LatLonCoords(lat, lon)); }

  /// Sets the coordinates from utm (and sets the latlon representation accordingly).
  void setUtm(double x, double y, const std::string& z) { set(UtmCoords(x,y,z)); }

  /// Sets the coordinates from utm (and sets the latlon representation accordingly).
  /// Does not change the zone.
  void setUtm(double x, double y) { set(UtmCoords(x,y,utm_.zone)); }

  /// Returns the utm coordinates.
  const UtmCoords& utm() const { return utm_; }

  /// Returns the latlon coordinates.
  const LatLonCoords& ll() const { return ll_; }


  /// @{ \name arithmetic operators

  /// Adds a displacement given in euclidean space.
  DualCoords& operator+= (const Eigen::Vector2d&);

  /// Substracts a displacement given in euclidean space.
  DualCoords& operator-= (const Eigen::Vector2d&);

  /// Adds a displacement given in euclidean space.
  const DualCoords operator+ (const Eigen::Vector2d& other) const {
    return DualCoords(*this) += other;
  }

  /// Substracts a displacement given in euclidean space.
  const DualCoords operator- (const Eigen::Vector2d& other) const {
    return DualCoords(*this) -= other;
  }

  /// @}
};



const global_coords::LatLonCoords center(const global_coords::LatLonCoords& ll1,
                                         const global_coords::LatLonCoords& ll2);

const global_coords::UtmCoords center(const global_coords::UtmCoords& u1,
                                      const global_coords::UtmCoords& u2);

const global_coords::DualCoords center(const global_coords::DualCoords& d1,
                                       const global_coords::DualCoords& d2);



}


/** @name Operator- (distance between points).

    @brief Computes the difference between two points. The returned vector is
    defined in the local UTM frame.

    @note Those functions work in the local UTM frame, and do not handle the
    case when different zones are involved. Even worst: there is no safe guard!
    So better use it other small distances.
  */
/// @{

const Eigen::Vector2d
operator- (const global_coords::LatLonCoords& lhs,
           const global_coords::LatLonCoords& rhs);


const Eigen::Vector2d
operator- (const global_coords::UtmCoords& lhs,
           const global_coords::UtmCoords& rhs);


const Eigen::Vector2d
operator- (const global_coords::DualCoords& lhs,
           const global_coords::DualCoords& rhs);

/// @}


#endif //__GLOBAL_COORDS_H__
