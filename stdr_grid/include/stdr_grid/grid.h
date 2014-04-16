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

#ifndef GRID_H_
#define GRID_H_

#include <ros/ros.h>


namespace stdr
{

/** \brief A non-rolling grid of cells: recentering is destructive.
  *
  * A cell can be accessed by:
  * - its row and column (RCLocal)
  * - its world coordinates (XY)
  * - its row and column offset from the cell at the origin of the world (RCGlobal)
  *
  * Notes:
  * - the row and column numbering starts in the lower left corner
  * - since coordinates are expressed as int16_t, the max value for the XY
  *   coordinates is 32767/resolution.
  */

template <typename T>
class Grid
{
private:
  double resolution_;              // map resolution in meters
  int16_t rows_, cols_;            // size of grid
  unsigned sz_;
  int16_t map_r0_, map_c0_;        // grid coordinates of lower left corner of map

  T* cell_;                        // actual map data
  T default_value_;                // default value for new cells

public:
  typedef T CellType;

  /// Constructs a grid with no cells
  Grid();

  /** \brief Constructs a grid with given resolution and dimensions.
    *
    * \param[in] resolution the size of a cell
    * \param[in] rows the number of rows
    * \param[in] cols the number of columns
    */
  Grid(double resolution, int16_t rows, int16_t cols);

  /** \brief Constructs a grid with given resolution and dimensions.
    *
    * \param[in] resolution the size of a cell
    * \param[in] rows the number of rows
    * \param[in] cols the number of columns
    * \param[in] default_value the default cell value to use
    */
  Grid(double resolution, int16_t rows, int16_t cols, const T & default_value);

  // copy constructor
  Grid(const Grid & other);

  // assignment operator
  Grid & operator= (const Grid & other);

  void setDimensions(double resolution, int16_t rows, int16_t cols);

  void setDimensions(double resolution, int16_t rows, int16_t cols, const T & default_value);

  ~Grid();

  inline double resolution() const { return resolution_; }
  inline int16_t rows() const { return rows_; }
  inline int16_t cols() const { return cols_; }

  /// Returns a pointer to the cell at world coordinates (x,y), NULL if (x,y)
  /// falls outside of the grid.
  template <class CT>
  T* getXY(CT x, CT y) const;

  /// Returns a pointer to the cell at global grid coordinates (number of rows
  /// and columns from the origin of the world coordinates, NULL if (r,c)
  /// falls outside of the grid.
  T* getRCGlobal(int16_t r, int16_t c) const;

  /// Returns a pointer to the cell at grid coordinates (number of rows
  /// and columns from the lower left corner). No boundary check!
  T* getRCLocalUnsafe(int16_t r, int16_t c) const;

  /// Returns a pointer to the n-th cell in the grid (starting from the lower
  /// left corner). No boundary check!
  /// Internally it computes the corresponding r,c and then calls getRCLocalUnsafe
  T* operator[] (unsigned n) const;

  /// Returns a pointer to the n-th cell in the grid (starting from the lower
  /// left corner). No boundary check!
  /// Internally it computes the corresponding r,c and then calls getRCLocalUnsafe
  T* at(unsigned n) const;

  /// Returns a pointer to the cell at grid coordinates (number of rows
  /// and columns from the lower left corner), NULL if (r,c)
  /// falls outside of the grid.
  T* getRCLocal(int16_t r, int16_t c) const;

  T* getCell(int n) const;

  /// Converts XY world coordinates into RCLocal coordinates.
  template <class CT>
  void xyToRCLocal(CT x, CT y, int16_t* r, int16_t* c) const;

  /// Converts RCLocal coordinates into world coordinates XY.
  template <class CT>
  void rcLocalToXY(int16_t r, int16_t c, CT* x, CT* y) const;

  /// Returns the RCLocal coordinates of the cell.
  void cellToRCLocal(const T* cell, int16_t* r, int16_t* c) const;

  /// Returns the world coordinates XY of the center of the cell.
  template <class CT>
  void cellToXY(const T* cell, CT* x, CT* y) const;

  /// Clears the grid by resetting each cell to the default value.
  void clear();

  /// Recenters the grid so that the new XY coordinates are placed at the center.
  /// Note that no copying or moving is done and this should be associated with
  /// clearing the grid.
  template <class CT>
  bool recenter(CT x, CT y);


private:

  /** \brief A fast version of floor()
   *
   * Floor is slow and casting is fast. But floor rounds to negative infinity
   * whereas the cast truncates towards zero. For negative numbers that will
   * give a different result, so we're adding a number larger than the
   * most-negative input we expect will be passed to this function,
   * then doing the cast.
   */
  static inline int32_t int_floor(double x) __attribute__ ((__const__));
};


template <typename T>
Grid<T>::Grid()
: resolution_(0), rows_(0), cols_(0), sz_(0),
  map_r0_(0), map_c0_(0), cell_(0)
{

}

template <typename T>
Grid<T>::Grid(double resolution, int16_t rows, int16_t cols)
  : resolution_(resolution), rows_(rows), cols_(cols), sz_(rows*cols)
  , map_r0_(0), map_c0_(0)
{
  cell_ = new T[sz_];
  clear();
}

template <typename T>
Grid<T>::Grid(double resolution, int16_t rows, int16_t cols, const T & default_value)
  : resolution_(resolution), rows_(rows), cols_(cols), sz_(rows*cols)
  , map_r0_(0), map_c0_(0)
{
  cell_ = new T[sz_];
  default_value_ = default_value;
  clear();
}

template <typename T>
Grid<T>::Grid(const Grid<T> & other)
{
  // copy grid properties
  resolution_ = other.resolution_;
  rows_ = other.rows_;
  cols_ = other.cols_;
  sz_ = other.sz_;
  map_r0_ = other.map_r0_;
  map_c0_ = other.map_c0_;
  default_value_ = other.default_value_;

  // allocate grid
  cell_ = new T[sz_];

  // copy grid cells
  for( unsigned n=0; n<sz_; ++n )
    cell_[n] = other.cell_[n];
}

template <typename T>
Grid<T> & Grid<T>::operator= (const Grid<T> & other)
{
  if( this!=&other ) {
    // reallocate grid if necessary
    if( sz_ != other.sz_ || cell_==0 ) {
      if( cell_ ) delete[] cell_;
      cell_ = new T[sz_];
    }

    // copy grid properties
    resolution_ = other.resolution_;
    rows_ = other.rows_;
    cols_ = other.cols_;
    sz_ = other.sz_;
    map_r0_ = other.map_r0_;
    map_c0_ = other.map_c0_;
    default_value_ = other.default_value_;

    // copy grid cells
    for( unsigned n=0; n<sz_; ++n )
      cell_[n] = other.cell_[n];
  }
  return *this;
}

template <typename T>
void Grid<T>::setDimensions(double resolution, int16_t rows, int16_t cols)
{
  resolution_ = resolution;
  rows_ = rows;
  cols_ = cols;
  sz_ = rows*cols;
  map_r0_ = 0;
  map_c0_ = 0;
  if(cell_) delete[] cell_;
  cell_ = new T[sz_];
  clear();
}

template <typename T>
void Grid<T>::setDimensions(double resolution, int16_t rows, int16_t cols, const T & default_value)
{
  resolution_ = resolution;
  rows_ = rows;
  cols_ = cols;
  sz_ = rows*cols;
  map_r0_ = 0;
  map_c0_ = 0;
  default_value_ = default_value;
  if(cell_) delete[] cell_;
  cell_ = new T[sz_];
  clear();
}

template <typename T>
Grid<T>::~Grid()
{
  if(cell_) delete[] cell_;
}

template <typename T>
template <class CT>
T* Grid<T>::getXY(CT x, CT y) const
{
  int16_t c = (int16_t) floor(x / resolution_) - map_c0_;
  int16_t r = (int16_t) floor(y / resolution_) - map_r0_;
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}
  return &cell_[r * cols_ + c];
}

template <typename T>
T* Grid<T>::getRCGlobal(int16_t r, int16_t c) const
{
  r -= map_r0_;
  c -= map_c0_;
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}
  return &cell_[r * cols_ + c];
}

template <typename T>
T* Grid<T>::getRCLocalUnsafe(int16_t r, int16_t c) const
{
  return &cell_[r * cols_ + c];
}

template <typename T>
T* Grid<T>::operator[] (unsigned n) const
{
  size_t c = n % cols_;
  size_t r = (n-c) / cols_;
  return getRCLocalUnsafe(r,c);
}

template <typename T>
T* Grid<T>::at(unsigned n) const
{
  size_t c = n % cols_;
  size_t r = (n-c) / cols_;
  return getRCLocalUnsafe(r,c);
}

template <typename T>
T* Grid<T>::getRCLocal(int16_t r, int16_t c) const
{
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}
  return &cell_[r * cols_ + c];
}

template <typename T>
T* Grid<T>::getCell(int n) const
{
  ROS_ASSERT(n>=0 && n<sz_);
  return cell_+n;
}

template <typename T>
template <class CT>
void Grid<T>::xyToRCLocal(CT x, CT y, int16_t* r, int16_t* c) const
{
  *c = int_floor(x / resolution_) - map_c0_;
  *r = int_floor(y / resolution_) - map_r0_;
}

template <typename T>
template <class CT>
void Grid<T>::rcLocalToXY(int16_t r, int16_t c, CT* x, CT* y) const
{
  *x = (map_c0_ + c + 0.5) * resolution_;
  *y = (map_r0_ + r + 0.5) * resolution_;
}

template <typename T>
void Grid<T>::cellToRCLocal(const T* cell, int16_t* r, int16_t* c) const
{
  int32_t n = cell - cell_;
  *r = n / cols_;
  *c = n - *r * cols_;
}

template <typename T>
template <class CT>
void Grid<T>::cellToXY(const T* cell, CT* x, CT* y) const
{
  int32_t n = cell - cell_;
  int16_t r = n / cols_;
  int16_t c = n - r * cols_;

  *x = (map_c0_ + c + 0.5) * resolution_;
  *y = (map_r0_ + r + 0.5) * resolution_;
}

template <typename T>
void Grid<T>::clear()
{
  for (unsigned i = 0; i < sz_; i++)
    cell_[i] = default_value_;
}

template <typename T>
template <class CT>
bool Grid<T>::recenter(CT x, CT y)
{
  map_r0_ = int_floor(y / resolution_) - rows_ / 2;
  map_c0_ = int_floor(x / resolution_) - cols_ / 2;
  return true;
}

template <typename T>
int32_t Grid<T>::int_floor(double x)
{
  return ((int32_t) (x + 100000.0)) - 100000;
}

} //namespace stdr

#endif
