cmake_minimum_required(VERSION 2.8.3)
project(yaml_cpp)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml_cpp
)

include_directories(include)

add_library(yaml_cpp
  src/binary.cpp
  src/convert.cpp
  src/directives.cpp
  src/emit.cpp
  src/emitfromevents.cpp
  src/emitter.cpp
  src/emitterstate.cpp
  src/emitterutils.cpp
  src/exp.cpp
  src/memory.cpp
  src/nodebuilder.cpp
  src/node.cpp
  src/node_data.cpp
  src/nodeevents.cpp
  src/null.cpp
  src/ostream_wrapper.cpp
  src/parse.cpp
  src/parser.cpp
  src/regex.cpp
  src/scanner.cpp
  src/scanscalar.cpp
  src/scantag.cpp
  src/scantoken.cpp
  src/simplekey.cpp
  src/singledocparser.cpp
  src/stream.cpp
  src/tag.cpp

  include/yaml-cpp/emitterdef.h
  include/yaml-cpp/mark.h
  include/yaml-cpp/traits.h
  include/yaml-cpp/eventhandler.h
  include/yaml-cpp/emitfromevents.h
  include/yaml-cpp/anchor.h
  include/yaml-cpp/emitter.h
  include/yaml-cpp/null.h
  include/yaml-cpp/dll.h
  include/yaml-cpp/node/detail/iterator_fwd.h
  include/yaml-cpp/node/detail/node_iterator.h
  include/yaml-cpp/node/detail/memory.h
  include/yaml-cpp/node/detail/iterator.h
  include/yaml-cpp/node/detail/node.h
  include/yaml-cpp/node/detail/node_ref.h
  include/yaml-cpp/node/detail/bool_type.h
  include/yaml-cpp/node/detail/impl.h
  include/yaml-cpp/node/detail/node_data.h
  include/yaml-cpp/node/emit.h
  include/yaml-cpp/node/convert.h
  include/yaml-cpp/node/type.h
  include/yaml-cpp/node/iterator.h
  include/yaml-cpp/node/node.h
  include/yaml-cpp/node/impl.h
  include/yaml-cpp/node/ptr.h
  include/yaml-cpp/node/parse.h
  include/yaml-cpp/emittermanip.h
  include/yaml-cpp/noncopyable.h
  include/yaml-cpp/contrib/anchordict.h
  include/yaml-cpp/contrib/graphbuilder.h
  include/yaml-cpp/parser.h
  include/yaml-cpp/ostream_wrapper.h
  include/yaml-cpp/binary.h
  include/yaml-cpp/yaml.h
  include/yaml-cpp/exceptions.h
  include/yaml-cpp/stlemitter.h
)
