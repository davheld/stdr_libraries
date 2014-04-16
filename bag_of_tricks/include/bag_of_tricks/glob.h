#include <glob.h>

inline std::vector<std::string> glob(const std::string& pattern)
{
  glob_t glob_result;
  glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
  std::vector<std::string> ret;
  for(unsigned int i = 0; i < glob_result.gl_pathc; ++i)
    ret.push_back(std::string(glob_result.gl_pathv[i]));
  globfree(&glob_result);
  return ret;
}
