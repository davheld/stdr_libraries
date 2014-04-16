/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.
  
  Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that the
  following conditions are met:
  
* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include<iostream>
#include<fstream>
#include<string>
#include<cstdlib>
#include <vector>
#include <queue>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <ros/assert.h>

#include <stdr_lib/roslib_helpers.h>

namespace bfs = boost::filesystem;


// from http://stackoverflow.com/questions/478898
std::string ssystem(const std::string& command, int *res=0)
{
  char tmpname [L_tmpnam];
  std::tmpnam ( tmpname );
  std::string cmd = command + " >> " + tmpname;
  int r = std::system(cmd.c_str());
  if( res )
    *res = r;
  std::ifstream file(tmpname, std::ios::in);
  std::string result;
  if (file) {
    while (!file.eof())
      result.push_back(file.get());
    file.close();
  }
  remove(tmpname);
  return result;
}


std::vector<bfs::path> list_dir(bfs::path p)
{
  std::vector<bfs::path> allfiles, files;
  std::queue<bfs::path> subdirs;
  subdirs.push(p);

  while( !subdirs.empty() )
  {
    p = subdirs.front();
    subdirs.pop();
    files.clear();
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    BOOST_FOREACH(const bfs::path& e, files) {
      if( bfs::is_regular_file(e) )
        allfiles.push_back(e);
      else if( bfs::is_directory(e) )
        subdirs.push(e);
    }
  }

  std::sort(allfiles.begin(), allfiles.end());
  return allfiles;
}


namespace stdr
{
namespace roslib
{


std::string find_file(const std::string& package, const std::string& filename)
{
  int result;
  const std::string command = std::string("catkin_find ") + package;
  std::string output = ssystem(command, &result);

  if( result!=0 ) {
    ROS_FATAL("command \'%s\' failed with error %d", command.c_str(), result);
    std::cout <<output <<std::endl;
    ROS_BREAK();
  }

  std::vector<std::string> lines;
  boost::algorithm::split(lines, output, boost::algorithm::is_any_of("\r\n"));
  ROS_ASSERT(lines.size()>0);

  // Each lines is a path. Search recursively for the desired file.
  std::vector<bfs::path> matching_paths;
  BOOST_FOREACH(bfs::path p, lines)
  {
    // although all should be a directory, the last line is usually a weird
    // single char. This might have to do with unicode chars... but I could not
    // find a better fix
    if( !bfs::is_directory(p) )
      continue;

    std::vector<bfs::path> files = list_dir(p);
    BOOST_FOREACH(const bfs::path& f, files) {
      if( boost::algorithm::ends_with(f.string(), filename) ) {
        ROS_DEBUG_STREAM("Found matching path " <<f);
        matching_paths.push_back(f);
      }
    }
  }
  if( matching_paths.empty() )
    return "";
  return matching_paths.front().string();
}


}
}
