#include <serializable/serializable.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace bfs = boost::filesystem;

// NDC = NoDefaultConstructor
//
// Typically, Serializable classes have a default ctor so you can create an
// uninitialized object, then call load() or deserialize() on it.
// 
// Classes which need to be deserialized but which cannot have a default ctor
// should implement an istream constructor.  I'd like to provide this
// by default via Serializable, but as far as I can tell this is not
// possible and you have to do it yourself.
//
// Fortunately it's pretty easy.
class NDC : public Serializable
{
public:
  int val_;
  
  NDC(int val) : val_(val) {}
  // This is pretty reasonable.
  NDC(istream& in) { deserialize(in); }
  // This isn't necessarily an OK thing to do.  Depends on the context.
  // If you do choose to use this form of deserialization constructor,
  // definitely make it explicit.
  explicit NDC(const std::string& path) { load(path); }
  
  void serialize(std::ostream& out) const
  {
    out.write((char*)&val_, sizeof(int));
  }
  
  void deserialize(std::istream& in)
  {
    cout << "NDC::deserialize" << endl;
    in.read((char*)&val_, sizeof(int));
  }
};

TEST(Serializable, ParentPath)
{
  bfs::path p("foo/bar/baz.txt");
  EXPECT_TRUE(p.has_parent_path());
  EXPECT_EQ(p.parent_path().string(), "foo/bar");
  EXPECT_TRUE(p.has_filename());
  EXPECT_EQ(p.filename().string(), "baz.txt");

  cout << "Parent path: " << p.parent_path().string() << endl;

  bfs::path p2("baz.txt");
  EXPECT_TRUE(!p2.has_parent_path());
}

TEST(Serializable, Serializable)
{
  int val = 13;
  NDC ndc(val);
  ndc.save("ndc");

  NDC ndc2((IfstreamWrapper("ndc")));
  cout << ndc2.val_ << endl;
  EXPECT_TRUE(ndc2.val_ == val);

  NDC ndc3("ndc");
  cout << ndc3.val_ << endl;
  EXPECT_TRUE(ndc3.val_ == val);

  if(!bfs::exists("tmp_directory"))
    bfs::create_directory("tmp_directory");
  ndc.save("tmp_directory/ndc");

  NDC ndc4((IfstreamWrapper("tmp_directory/ndc")));
  cout << ndc4.val_ << endl;
  EXPECT_TRUE(ndc4.val_ == val);
}

TEST(ThreadedSerializer, SerializableObject)
{
  ThreadedSerializer<NDC> ser;
  ser.verbose_ = true;
  ThreadPtr thread = ser.launch();  // Run in a different thread.

  string dir = "threaded_serializer_test";
  if(!bfs::exists(dir))
    bfs::create_directory(dir);

  int num = 100;
  for(int i = 0; i < num; ++i) {
    ostringstream oss;
    oss << dir << "/ndc" << setw(3) << setfill('0') << i;
    ser.push(NDC(i), oss.str());
  }

  ser.quit(); // Finish serializing everything in your queue and shut down your thread.
  thread->join();
  cout << "Done." << endl;
}

struct CustomSerializationFunction
{
  void operator()(std::string str, const std::string& path) {
    ofstream f;
    f.open(path.c_str());
    if(!f.is_open()) {
      cerr << "Failed to open " << path << endl;
      assert(f.is_open());
    }
    f << str << endl;
    f.close();
  }
};

TEST(ThreadedSerializer, CustomSerializationFunction)
{
  ThreadedSerializer<std::string, CustomSerializationFunction> ser;
  ser.verbose_ = true;
  ThreadPtr thread = ser.launch();  // Run in a different thread.

  string dir = "custom_threaded_serializer_test";
  if(!bfs::exists(dir))
    bfs::create_directory(dir);

  int num = 100;
  for(int i = 0; i < num; ++i) {
    ostringstream oss;
    oss << dir << "/string" << setw(3) << setfill('0') << i;
    string path = oss.str();
    oss.str("");
    oss << "The number is " << i << endl;
    string str = oss.str();
    ser.push(str, path);
  }

  ser.quit(); // Finish serializing everything in your queue and shut down your thread.
  thread->join();
  cout << "Done." << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

