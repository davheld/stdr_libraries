#include <matplotlib_interface/matplotlib_interface.h>

using namespace Eigen;
using namespace std;

int main(int argc, char** argv) 
{
  FILE *fp = fopen("example_gaussian.py", "r");
  if( !fp ) {
    printf("ERROR: this program must be run from the root of the matplotlib_interface package\n");
    printf("       i.e. where example_gaussian.py can be found in the working directory\n");
    exit(1);
  }
  fclose(fp);
  
  printf(" --- INFO --- Initialize use of matplotlib-interface.\n");
  mpliBegin();
  
  printf(" --- INFO --- Hello world with python.\n");
  mpli("print 'This is python speaking.\\n'");

  printf(" --- INFO --- Export a double into python as a numpy array with size one.\n");
  double dbl = 42.42;
  mpliExport(dbl); // A numpy array with the name 'dbl' now exists in python.
  cout << "dbl in C++: " << dbl << endl << endl;
  mpli("print 'dbl in python: '");
  mpli("print dbl");
  cout << endl;
  
  printf(" --- INFO --- Export an Eigen::VectorXd or Eigen::MatrixXd into python.\n");
  VectorXd vec = VectorXd::Random(3);
  MatrixXd mat = MatrixXd::Random(3, 3);
  mpliExport(vec); // A numpy array with the name 'vec' now exists in python.
  mpliExport(mat); // A numpy array with the name 'mat' now exists in python.

  cout << "vec in C++: " << endl << vec << endl << endl;
  mpli("print 'vec in python: '");
  mpli("print vec");
  cout << endl;
  mpli("print 'vec.transpose() in python.  Note that unlike with Eigen, there is no difference - it is a 1d array:'");
  mpli("print vec.transpose()");
  cout << endl;

  cout << "mat in C++: " << endl << mat << endl << endl;
  mpli("print 'mat in python: '");
  mpli("print mat");
  cout << endl;
  
  printf(" --- INFO --- Export a vector or matrix with a new name.\n");
  VectorXd *rand = new VectorXd(2);
  *rand = VectorXd::Random(2);
  mpliNamedExport("new_name", *rand); // mpliExport(*rand) won't work because of the dereferencing *, so use mpliNamedExport instead.
  mpli("print 'new_name in python:'");
  mpli("print new_name");
  cout << endl;
  
  cout << "Press return to show some example plots." << endl;
  cin.ignore();
  mpli("from pylab import *");
  
  printf(" --- INFO --- Simple plot demo.\n");
  VectorXd x(100);
  for(int i = 0; i < x.rows(); ++i) {
    x(i) = i*i;
  }
  mpliExport(x);
  mpli("plot(x)");
  mpli("draw()");
  mpli("waitforbuttonpress()");
  mpli("clf()");

  printf(" --- INFO --- Execute an entire file.  Export variables in c++ that describe a Gaussian, then display it with mpli.\n");
  double var_x = 1;
  double var_y = 1;
  double var_xy = 0.5;
  double mu_x = 1;
  double mu_y = 2;
  mpliExport(var_x);
  mpliExport(var_y);
  mpliExport(var_xy);
  mpliExport(mu_x);
  mpliExport(mu_y);
  mpliExecuteFile("example_gaussian.py");

  printf(" --- INFO --- Histogram demo.\n");
  VectorXd samples(10000);
  for(int i = 0; i < samples.rows(); ++i)
    samples(i) = 0.5 * VectorXd::Random(12).sum(); // Sample from normal distribution with mean 0, var 1.
  mpliExport(samples);
  mpli("hist(samples, 50, facecolor='green', alpha=0.75)");
  mpli("draw()");
  mpli("waitforbuttonpress()");
  mpli("clf()");
  
  mpliEnd();

  return 0;
}

void numpyMultipleImportTest() {
  // -- The Py_Initialize, import numpy, Py_Finalize loop segfaults the second time through due to a bug in numpy.
  //    If you remove the finalize call, it looks like it's only a constant and small memory leak.
  for(int i = 0; i < 20; ++i) {
    cout << i << endl;
    Py_Initialize();
    mpli("'print hi...'");
    mpli("import numpy");
    //Py_Finalize();
  }  
}
