#include <eigen_extensions/random.h>

using namespace std;
using namespace Eigen;

namespace eigen_extensions
{

  UniformSampler::UniformSampler(uint64_t seed) :
    mersenne_(seed)
  {
  }

  uint64_t UniformSampler::sample()
  {
    return mersenne_();
  }

  GaussianSampler::GaussianSampler(double mean, double stdev, uint64_t seed) :
    Sampler(),
    mersenne_(seed),
    normal_(mean, stdev)
#if !( defined(__MACH__) && defined(__APPLE__) )
    , vg_(mersenne_, normal_)
#endif
  {
  }

  double GaussianSampler::sample()
  {
#if defined(__MACH__) && defined(__APPLE__)
    return normal_(mersenne_);
#else
    return vg_();
#endif
  }

  void sampleSparseGaussianVector(int rows, int nnz, SparseVector<double>* vec)
  {
    assert(rows >= nnz);

    vector<int> indices(rows);
    for(size_t i = 0; i < indices.size(); ++i)
      indices[i] = i;
    random_shuffle(indices.begin(), indices.end());

    GaussianSampler gs;
    *vec = SparseVector<double>(rows);
    vec->reserve(nnz);
    for(int i = 0; i < nnz; ++i)
      vec->coeffRef(indices[i]) = gs.sample();
  }

  int weightedSample(Eigen::VectorXd weights)
  {
    double inv_sum = 1.0 / weights.sum();
    double r = (double)rand() / (double)RAND_MAX;
    
    double cumulative = 0;
    for(int i = 0; i < weights.rows(); ++i) {
      cumulative += weights(i) * inv_sum;
      if(cumulative >= r)
	return i;
    }
    return weights.rows() - 1;
  }

  void weightedSample(Eigen::VectorXd weights, Eigen::VectorXi* indices)
  {
    assert(indices->rows() > 0);
    for(int i = 0; i < indices->rows(); ++i)
      indices->coeffRef(i) = weightedSample(weights);
  }

  void weightedSampleLowVariance(Eigen::VectorXd weights, std::mt19937* mersenne, Eigen::VectorXi* indices)
  {
    assert(indices->rows() > 0);

    weights /= weights.sum();
    double r = ((double)(*mersenne)() / mersenne->max()) / indices->rows();
    double c = weights(0);
    int i = 0;
    for(int m = 0; m < indices->rows(); ++m) {
      double u = r + m / (double)indices->rows();
      while(u > c) {
	++i;
	i = min(i, (int)(weights.rows() - 1));  // edge case
	c += weights(i);
      }
      indices->coeffRef(m) = i;
    }
  }

  
} // namespace

