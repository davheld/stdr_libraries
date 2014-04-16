#include <boost/filesystem.hpp>
#include <ros/assert.h>

using namespace std;
namespace bfs = boost::filesystem;

std::string nextPath(std::string dir, std::string prefix, std::string suffix, int padding)
{
  ROS_ASSERT(bfs::exists(dir));

  // -- Find the max number in the directory.
  int max_num = -1;
  bfs::directory_iterator end_itr;  // default construction yields past-the-end
  for(bfs::directory_iterator itr(dir); itr != end_itr; ++itr) {
    string filename = itr->path().leaf().string();
    if(filename.substr(0, prefix.size()) != prefix)
      continue;
    if(filename.substr(filename.size() - suffix.size()) != suffix)
      continue;

    string numstr = filename.substr(prefix.size(), filename.size() - prefix.size() - suffix.size());
    int num = atoi(numstr.c_str());
    max_num = max(max_num, num);
  }

  ostringstream oss;
  oss << dir << "/" << prefix << setw(padding) << setfill('0') << max_num + 1 << suffix;
  ROS_ASSERT(!bfs::exists(oss.str()));
  
  return oss.str();
}
