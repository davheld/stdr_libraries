#include <boost/filesystem.hpp>
#include <serializable/serializable.h>

using namespace std;
namespace bfs = boost::filesystem;

void Serializable::load(const std::string& path)
{
  ifstream f;
  f.open(path.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }
  deserialize(f);
  f.close();
}

void Serializable::save(const std::string& path) const
{
  // -- Get a temporary file to save to.
  bfs::path p(path);
  assert(p.has_filename());
  string tmppath;
  if(p.has_parent_path())
    tmppath = p.parent_path().string() + "/";
  else
    tmppath = "./";
  tmppath = tmppath + ".Serializable-" + p.filename().string();

  // -- Write to disk.
  ofstream f;
  f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  f.open(tmppath.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }

  try {
    serialize(f);
  }
  catch(std::ios_base::failure& e) {
    const bfs::space_info s = bfs::space(bfs::path(tmppath));
    cerr <<"Serialize failed. The disk might be full (" <<s.available <<" available out of " <<s.capacity <<")." <<endl;
    cerr.flush();
    abort();
  }

  f.close();

  // -- Move the temporary file to the intended destination.
  bfs::rename(tmppath, path);
}

std::ostream& operator<<(std::ostream& out, const Serializable& ser)
{
  ser.serialize(out);
  return out;
}

std::istream& operator>>(std::istream& in, Serializable& ser)
{
  ser.deserialize(in);
  return in;
}

void YAMLizable::loadYAML(const std::string& path)
{
  YAML::Node doc = YAML::LoadFile(path);
  deYAMLize(doc);
}

void YAMLizable::saveYAML(const std::string& path) const
{
  ::saveYAML(YAMLize(), path);
}

IfstreamWrapper::IfstreamWrapper(const std::string& path)
{
  ifstream_.open(path.c_str());
  if(!ifstream_.is_open()) {
    cerr << "Failed to open " << path << endl;
  }
  assert(ifstream_.is_open());
}

IfstreamWrapper::~IfstreamWrapper()
{
  if(ifstream_.is_open())
    ifstream_.close();
}

void saveYAML(const YAML::Node& doc, const std::string& path)
{
  ofstream f;
  f.open(path.c_str());
  if(!f.is_open()) {
    cerr << "Failed to open " << path << endl;
    assert(f.is_open());
  }
  
  f << YAML::Dump(doc) << endl;
  f.close();
}
