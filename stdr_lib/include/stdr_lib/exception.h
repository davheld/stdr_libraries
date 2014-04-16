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


#ifndef __STDR_EXCEPTIONS__EXCEPTION_H__
#define __STDR_EXCEPTIONS__EXCEPTION_H__

#include <string>

#include <boost/exception/all.hpp>
#include <boost/throw_exception.hpp>

#include <ros/ros.h>

namespace stdr {

namespace ex {
/** @{
  * \brief Exceptions.
  *
  * Based on boost exceptions.
  */


// some error info types

/// Addition message info for the exception
typedef ::boost::error_info<struct tag_err_msg,std::string> MsgInfo;

/* Also you can use the standard errinfo types defined by boost:
  see http://www.boost.org/doc/libs/1_54_0/libs/exception/doc/error_info.html

  - errinfo_api_function
  - errinfo_at_line
  - errinfo_errno
  - errinfo_file_handle
  - errinfo_file_name
  - errinfo_file_open_mode
  - errinfo_nested_exception
  - errinfo_type_info_name
*/


/// A base exception for all exception thrown by our code
struct ExceptionBase: virtual ::std::exception, virtual ::boost::exception
{
  ExceptionBase() {}
  ExceptionBase(const std::string & msg)
  { *this << MsgInfo(msg); }
};



// Common specialized exceptions


/// @{
/// Exceptions for io errors

/// Base exception for io errors
struct IOError: virtual ExceptionBase { };
/// end of file reached
struct EOFError: virtual IOError { };
/// @}

/// @{
/// Exceptions for logic errors (bad arguments, bad conditions, etc.)

/// Base exception for logic errors
struct LogicError: virtual ExceptionBase { };
/// @}


/// Exception for when a parameter cannot be found in the rosparam server.
struct RosparamError : virtual ExceptionBase {
  RosparamError() {}
  RosparamError(const std::string param_name)
  { *this << MsgInfo(std::string("Could not get parameter ")+param_name); }
  RosparamError(const ros::NodeHandle & nh, const std::string param_name)
  { *this << MsgInfo(std::string("Could not get parameter ")+nh.getNamespace()+"/"+param_name); }
};



/// @}
} //namespace ex

} // namespace stdr_ex

#endif //__STDR_EXCEPTIONS__EXCEPTION_H__

