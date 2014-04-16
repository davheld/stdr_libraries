#ifndef SERIALIZABLE_H
#define SERIALIZABLE_H

#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include <queue>
#include <map>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/node.h>
#include <agent/agent.h>


/** \brief @b Serializable is an abstract base class which represents
 * objects that can be serialized and deserialized.
 *
 * See test_serializable.cpp for example usage.
 */
class Serializable
{
public:
  virtual ~Serializable() {};
  virtual void serialize(std::ostream& out) const = 0;
  virtual void deserialize(std::istream& in) = 0;
  virtual void save(const std::string& path) const;
  virtual void load(const std::string& path);
};

std::ostream& operator<<(std::ostream& out, const Serializable& ser);
std::istream& operator>>(std::istream& in, Serializable& ser);

class YAMLizable
{
public:
  virtual ~YAMLizable() {};
  virtual YAML::Node YAMLize() const = 0;
  virtual void deYAMLize(const YAML::Node& in) = 0;
  virtual void saveYAML(const std::string& path) const;
  virtual void loadYAML(const std::string& path);
};

//! Really, this doesn't exist in yaml-cpp?  Seriously?
void saveYAML(const YAML::Node& doc, const std::string& path);

/* Class for making it easy to use stream constructors.
 * For example, assume OnlineLearner doesn't have a default constructor,
 * but does have a OnlineLearner(istream& in) constructor.
 * Then to load a new one from disk, you can do this:
 * OnlineLearner learner((IfstreamWrapper(path)));
 * Note the extra parens.  They are unfortunately necessary due to the
 * possibility that you are declaring a function "learner" that
 * takes an IfstreamWrapper called path and returns an OnlineLearner.
 * Really, C++?  Why?  It's almost perfect.  Alas.
 */
class IfstreamWrapper
{
public:
  IfstreamWrapper(const std::string& path);
  ~IfstreamWrapper();
  operator std::istream&() { return ifstream_; }
  
private:
  std::ifstream ifstream_;
};

struct SerializableSaveFunction
{
  void operator()(const Serializable& obj, const std::string& path) { obj.save(path); }
};

template<typename T, typename S = SerializableSaveFunction>
class ThreadedSerializer : public Agent
{
public:
  double delay_;
  bool verbose_;

  ThreadedSerializer() : delay_(10), verbose_(false) {}

  void push(T obj, const std::string& path)
  {
    scopeLockWrite; queue_.push(std::pair<T, std::string>(obj, path));
  }

  void _run()
  {
    S serialize;
    while(true) {
      usleep(delay_ * 1e3);
      scopeLockWrite;
      if(!queue_.empty()) {
        serialize(queue_.front().first, queue_.front().second);
        if(verbose_)
          std::cout << "[ThreadedSerializer]  Saved to \""
                    << queue_.front().second << "\"." << std::endl;
        queue_.pop();
      }

      if(queue_.empty() && quitting_)
        break;
    }
  }
  
protected:
  std::queue< std::pair<T, std::string> > queue_;
};

#endif // SERIALIZABLE_H
