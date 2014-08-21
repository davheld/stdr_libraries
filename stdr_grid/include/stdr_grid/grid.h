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
  */

template <typename T>
class Grid
{
private:
  double resolution_;    // map resolution in meters
  unsigned rows_, cols_; // size of grid
  unsigned long sz_;     // total number of cells
  double x0_, y0_;       // world coordinates of first cell (lower left)

  T* cell_;              // actual map data
  T default_value_;      // default value for new cells

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
  Grid(double resolution, unsigned rows, unsigned cols);

  /** \brief Constructs a grid with given resolution and dimensions.
    *
    * \param[in] resolution the size of a cell
    * \param[in] rows the number of rows
    * \param[in] cols the number of columns
    * \param[in] default_value the default cell value to use
    */
  Grid(double resolution, unsigned rows, unsigned cols, const T & default_value);

  // copy constructor
  Grid(const Grid & other);

  // assignment operator
  Grid & operator= (const Grid & other);

  void setDimensions(double resolution, unsigned rows, unsigned cols);

  void setDimensions(double resolution, unsigned rows, unsigned cols, const T & default_value);

  ~Grid();

  inline double resolution() const { return resolution_; }
  inline unsigned rows() const { return rows_; }
  inline unsigned cols() const { return cols_; }


  /// Returns a pointer to the n-th cell in the grid (starting from the lower
  /// left corner). No boundary check!
  T* operator[] (size_t n) const;

  /// Returns a pointer to the n-th cell in the grid (starting from the lower
  /// left corner). No boundary check!
  T* at(size_t n) const;

  /// Returns a pointer to the n-th cell in the grid (starting from the lower
  /// left corner). No boundary check!
  T* getCell(size_t n) const;


  /// Returns a pointer to the cell at grid coordinates (number of rows
  /// and columns from the lower left corner). No boundary check!
  T* getRCLocalUnsafe(int r, int c) const;

  /// Returns a pointer to the cell at grid coordinates (number of rows
  /// and columns from the lower left corner), NULL if (r,c)
  /// falls outside of the grid.
  T* getRCLocal(int r, int c) const;

  /// Returns a pointer to the cell at global grid coordinates (number of rows
  /// and columns from the origin of the world coordinates, NULL if (r,c)
  /// falls outside of the grid.
  T* getRCGlobal(int r, int c) const;


  /// Converts XY world coordinates into RCLocal coordinates.
  void xyToRCLocal(double x, double y, int* r, int* c) const;

  /// Converts RCLocal coordinates into world coordinates XY.
  void rcLocalToXY(int r, int c, double* x, double* y) const;

  /// Returns the RCLocal coordinates of the cell.
  void cellToRCLocal(const T* cell, int* r, int* c) const;

  /// Returns the world coordinates XY of the center of the cell.
  void cellToXY(const T* cell, double* x, double* y) const;


  /// Returns a pointer to the cell at world coordinates (x,y), NULL if (x,y)
  /// falls outside of the grid.
  T* getXY(double x, double y) const;


  /// Clears the grid by resetting each cell to the default value.
  void clear();

  /// Recenters the grid so that the new XY coordinates are placed at the center.
  /// Note that no copying or moving is done and this should be associated with
  /// clearing the grid.
  bool recenter(double x, double y);


private:

  /** \brief A fast version of floor()
   *
   * Floor is slow and casting is fast. But floor rounds to negative infinity
   * whereas the cast truncates towards zero. For negative numbers that will
   * give a different result, so we're adding a number larger than the
   * most-negative input we expect will be passed to this function,
   * then doing the cast.
   */
  static inline int int_floor(double x) __attribute__ ((__const__));
};


template <typename T>
Grid<T>::Grid()
: resolution_(0), rows_(0), cols_(0), sz_(0),
  x0_(0), y0_(0), cell_(0)
{

}

template <typename T>
Grid<T>::Grid(double resolution, unsigned rows, unsigned cols)
  : resolution_(resolution), rows_(rows), cols_(cols)
  , x0_(0), y0_(0)
{
  ROS_ASSERT( rows <= std::numeric_limits<int>::max() );
  ROS_ASSERT( cols <= std::numeric_limits<int>::max() );
  sz_ = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  cell_ = new T[sz_];
  clear();
}

template <typename T>
Grid<T>::Grid(double resolution, unsigned rows, unsigned cols, const T & default_value)
  : resolution_(resolution), rows_(rows), cols_(cols)
  , x0_(0), y0_(0)
{
  ROS_ASSERT( rows <= std::numeric_limits<int>::max() );
  ROS_ASSERT( cols <= std::numeric_limits<int>::max() );
  sz_ = static_cast<size_t>(rows) * static_cast<size_t>(cols);
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
  x0_ = other.x0_;
  y0_ = other.y0_;
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
    x0_ = other.x0_;
    y0_ = other.y0_;
    default_value_ = other.default_value_;

    // copy grid cells
    for( unsigned n=0; n<sz_; ++n )
      cell_[n] = other.cell_[n];
  }
  return *this;
}

template <typename T>
void Grid<T>::setDimensions(double resolution, unsigned rows, unsigned cols)
{
  ROS_ASSERT( rows <= std::numeric_limits<int>::max() );
  ROS_ASSERT( cols <= std::numeric_limits<int>::max() );

  resolution_ = resolution;
  rows_ = rows;
  cols_ = cols;
  sz_ = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  x0_ = 0;
  y0_ = 0;
  if(cell_)
    delete[] cell_;
  cell_ = new T[sz_];
  clear();
}

template <typename T>
void Grid<T>::setDimensions(double resolution, unsigned rows, unsigned cols, const T & default_value)
{
  ROS_ASSERT( rows <= std::numeric_limits<int>::max() );
  ROS_ASSERT( cols <= std::numeric_limits<int>::max() );

  resolution_ = resolution;
  rows_ = rows;
  cols_ = cols;
  sz_ = static_cast<size_t>(rows) * static_cast<size_t>(cols);
  x0_ = 0;
  y0_ = 0;
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
T* Grid<T>::getCell(size_t n) const
{
  return cell_+n;
}

template <typename T>
T* Grid<T>::operator[] (size_t n) const
{
  return getCell(n);
}

template <typename T>
T* Grid<T>::at(size_t n) const
{
  return getCell(n);
}

template <typename T>
T* Grid<T>::getRCLocalUnsafe(int r, int c) const
{
  return getCell(r * cols_ + c);
}

template <typename T>
T* Grid<T>::getRCLocal(int r, int c) const
{
  if (r < 0 || c < 0 || r >= rows_ || c >= cols_)
    return NULL;
  return getRCLocalUnsafe(r, c);
}

template <typename T>
void Grid<T>::xyToRCLocal(double x, double y, int* r, int* c) const
{
  *c = int_floor( (x-x0_) / resolution_ );
  *r = int_floor( (y-y0_) / resolution_ );
}

template <typename T>
void Grid<T>::rcLocalToXY(int r, int c, double* x, double* y) const
{
  *x = c * resolution_ + x0_;
  *y = r * resolution_ + y0_;
}

template <typename T>
void Grid<T>::cellToRCLocal(const T* cell, int* r, int* c) const
{
  size_t n = cell - cell_;
  *r = n / cols_;
  *c = n - *r * cols_;
}

template <typename T>
void Grid<T>::cellToXY(const T* cell, double* x, double* y) const
{
  int r, c;
  cellToRCLocal(cell, &r, &c);
  rcLocalToXY(r, c, x, y);
}

template <typename T>
T* Grid<T>::getXY(double x, double y) const
{
  int r, c;
  xyToRCLocal(x, y, &r, &c);
  return getRCLocal(r, c);
}

template <typename T>
T* Grid<T>::getRCGlobal(int r, int c) const
{
  r = int_floor( (r*resolution_ - y0_) / resolution_ );
  c = int_floor( (c*resolution_ - x0_) / resolution_ );
  return getRCLocal(r, c);
}

template <typename T>
void Grid<T>::clear()
{
  for (unsigned i = 0; i < sz_; i++)
    cell_[i] = default_value_;
}

template <typename T>
bool Grid<T>::recenter(double x, double y)
{
  x0_ = x - static_cast<double>(cols_)/2 * resolution_;
  y0_ = y - static_cast<double>(rows_)/2 * resolution_;
  return true;
}

template <typename T>
int Grid<T>::int_floor(double x)
{
  return ((int) (x + 100000.0)) - 100000;
}

} //namespace stdr

#endif
