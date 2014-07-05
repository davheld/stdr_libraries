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

#ifndef ROSPARAM_HELPERS_H
#define ROSPARAM_HELPERS_H

#include <ros/ros.h>
#include <stdr_lib/exception.h>


/** Some macros to help working with the rosparameter server.
  *
  * These are implemented as macros, as opposed to functions, so that the
  * rosconsole message provenance is the calling function, and not the helper
  * function.
  */


/* ============================================================================
 * Some helper macros, not meant to be used outside of this file.
 * ===========================================================================*/

#define __ROS_PARAM_ERR_MSG__(nh, key) "Could not get parameter " <<(nh).resolveName(key) <<'.'
#define __ROS_PARAM_ERR_MSG_DEF__(nh, key, def) "Could not get parameter " <<(nh).resolveName(key) <<". Using default: " <<(def)

#define __GET_ROS_PARAM__(nh, param, value, default_value, lvl) do { \
  if( ! (nh).getParam(param, value) ) { \
    value = (default_value); \
    ROS_ ## lvl ## _STREAM( __ROS_PARAM_ERR_MSG_DEF__(nh, param, default_value) ); \
  } \
} while(0)

#define __GET_ROS_PARAM_ONCE__(nh, param, value, default_value, lvl) do { \
  if( ! (nh).getParam(param, value) ) { \
    value = (default_value); \
    ROS_ ## lvl ## _STREAM_ONCE( __ROS_PARAM_ERR_MSG_DEF__(nh, param, default_value) ); \
  } \
} while(0)


//!@{
/**
  * Gets a rosparam. If not found, prints with ros console, and applies the
  * default value.
  */
#define GET_ROS_PARAM_DEBUG(nh, param, value, default_value) __GET_ROS_PARAM__(nh, param, value, default_value, DEBUG)
#define GET_ROS_PARAM_DEBUG_ONCE(nh, param, value, default_value) __GET_ROS_PARAM_ONCE__(nh, param, value, default_value, DEBUG)
#define GET_ROS_PARAM_INFO(nh, param, value, default_value) __GET_ROS_PARAM__(nh, param, value, default_value, INFO)
#define GET_ROS_PARAM_INFO_ONCE(nh, param, value, default_value) __GET_ROS_PARAM_ONCE__(nh, param, value, default_value, INFO)
#define GET_ROS_PARAM_WARN(nh, param, value, default_value) __GET_ROS_PARAM__(nh, param, value, default_value, WARN)
#define GET_ROS_PARAM_WARN_ONCE(nh, param, value, default_value) __GET_ROS_PARAM_ONCE__(nh, param, value, default_value, WARN)
//!@}


/**
  * Gets a rosparam. If not found, prints with ros console, and aborts.
  */
#define GET_ROS_PARAM_ABORT(nh, param, value) do { \
  if( ! (nh).getParam(param, value) ) { \
    ROS_FATAL_STREAM( __ROS_PARAM_ERR_MSG__(nh, param) ); \
    ROS_BREAK(); \
  } \
} while(0)


/**
  * Gets a rosparam. If not found, prints with ros console, and throws a
  * stdr::ex::RosparamError exception.
  */
#define GET_ROS_PARAM_EX(nh, param, value) do { \
  if( ! (nh).getParam(param, value) ) { \
    ROS_ERROR_STREAM( __ROS_PARAM_ERR_MSG__(nh, param) ); \
    BOOST_THROW_EXCEPTION(stdr::ex::RosparamError(nh, param)); \
  } \
} while(0)


namespace stdr {

///@{
/** These functions return a parameter from the ros parameter server.
  *
  * Compared to the GET_ROS_PARAM_XXX macros, these functions return a value
  * (as opposed to set the value of an existing variable), which is useful
  * when dealing with a const variable. On the other hand, the error reporting
  * is less useful, as the ros console macros will report the error as coming
  * from this file, instead of the place where it was called.
  */

/** Returns the value of the parameter. Crashes if it does not exist.
  */
template <typename T>
T get_rosparam(const ros::NodeHandle& nh, const std::string& param)
{
  T param_val;
  GET_ROS_PARAM_ABORT(nh, param, param_val);
  return param_val;
}

/** Returns the value of the parameter if it exists, otherwise returns @param default_val.
  */
template <typename T>
T get_rosparam(const ros::NodeHandle& nh, const std::string& param, const T& default_val)
{
  T param_val;
  GET_ROS_PARAM_WARN(nh, param, param_val, default_val);
  return param_val;
}

///@}

} //namespace stdr

#endif // ROSPARAM_HELPERS_H
