#include <bag_of_tricks/next_path.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <ros/assert.h>

using namespace std;
namespace bfs = boost::filesystem;

TEST(nextPath, nextPath)
{
  string dir = "aoeu";
  string prefix = "prefix-";
  string suffix = "-suffix.txt";
  int padding = 4;
  int retval;
  bfs::remove_all(dir);
  bfs::create_directory(dir);

  for(int i = 0; i < 42; ++i) { 
    string path = nextPath(dir, prefix, suffix, padding);
    cout << "Creating path " << path << endl;
    retval = system(("touch " + path).c_str());
    ROS_ASSERT(retval == 0);
  }

  retval = system(("rm " + dir + "/" + prefix + "0033" + suffix).c_str());
  ROS_ASSERT(retval == 0);

  for(int i = 0; i < 5; ++i) { 
    string path = nextPath(dir, prefix, suffix, padding);
    cout << "Creating path " << path << endl;
    retval = system(("touch " + path).c_str());
    ROS_ASSERT(retval == 0);
  }

  EXPECT_TRUE(bfs::exists(dir + "/" + prefix + "0046" + suffix));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
