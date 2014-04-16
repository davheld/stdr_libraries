#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <timer/timer.h>

/** \brief Tests whether int_floor() is really faster than floor().
 *
 * int_floor is used in the grid code in place of floor, as it's supposedly
 * faster.
 */



/** \brief A fast version of floor()
 *
 * Floor is slow and casting is fast. But floor rounds to negative infinity
 * whereas the cast truncates towards zero. For negative numbers that will
 * give a different result, so we're adding a number larger than the
 * most-negative input we expect will be passed to this function,
 * then doing the cast.
 */
inline int int_floor(double x) __attribute__ ((__const__));
int int_floor(double x) { return ((int) (x + 100000.0)) - 100000; }


int main()
{
  const unsigned N = 10000;
  double numbers[N];

  for( double * n = numbers; n<numbers+N; ++n )
    *n = (drand48() - 0.5) * 1000;

  {
    ScopedTimer int_floor_timer("int_floor", TimeUnit::MS);
    int result = 0;
    for( unsigned i = 0; i<N; ++i )
      for( const double * n = numbers; n<numbers+N; ++n )
        result += int_floor(*n);
    printf("result=%d\n", result);
  }

  {
    ScopedTimer floor_timer("floor", TimeUnit::MS);
    int result = 0;
    for( unsigned i = 0; i<N; ++i )
      for( const double * n = numbers; n<numbers+N; ++n )
        result += floor(*n);
    printf("result=%d\n", result);
  }

  return 0;
}
