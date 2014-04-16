#ifndef EIGEN_EXTENSIONS_RANDOM_H
#define EIGEN_EXTENSIONS_RANDOM_H

#include <stdint.h>
#if defined(__MACH__) && defined(__APPLE__)
#include <random>
#include <functional>
#else
#include <tr1/random>
namespace std {
  using std::tr1::mt19937;
  using std::tr1::normal_distribution;
  using std::tr1::variate_generator;
};
#endif
#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif
#include <Eigen/Eigen>
#include <Eigen/Sparse>

namespace eigen_extensions
{

class Sampler
{
public:
  virtual ~Sampler() {}
  virtual double sample() = 0;
};

class UniformSampler
{
public:
  UniformSampler(uint64_t seed = 0);
  uint64_t sample();

protected:
  std::mt19937 mersenne_;
};

class GaussianSampler : public Sampler
{
public:
  GaussianSampler(double mean = 0, double variance = 1, uint64_t seed = 0);
  double sample();
  template<class S, int T, int U> void sample(Eigen::Matrix<S, T, U>* mat);

protected:
  std::mt19937 mersenne_;
  std::normal_distribution<double> normal_;
#if !( defined(__MACH__) && defined(__APPLE__) )
  std::variate_generator<std::mt19937, std::normal_distribution<double> > vg_;
#endif
};

template<class S, int T, int U>
void GaussianSampler::sample(Eigen::Matrix<S, T, U>* mat)
{
  for(int i = 0; i < mat->cols(); ++i)
    for(int j = 0; j < mat->rows(); ++j)
      mat->coeffRef(j, i) = sample();
}

template<class S, int T, int U>
void sampleGaussian(Eigen::Matrix<S, T, U>* mat)
{
  GaussianSampler gs;
  gs.sample(mat);
}

void sampleSparseGaussianVector(int rows, int nnz, Eigen::SparseVector<double>* vec);
int weightedSample(Eigen::VectorXd weights);
//! Fills indices with samples from the weights vector, with replacement.
void weightedSample(Eigen::VectorXd weights, Eigen::VectorXi* indices);
//! Uses mersenne for generating random numbers.
void weightedSampleLowVariance(Eigen::VectorXd weights,
                               std::mt19937* mersenne,
                               Eigen::VectorXi* indices);
}


#endif // EIGEN_EXTENSIONS_RANDOM_H
