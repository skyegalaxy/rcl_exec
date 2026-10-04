// Copyright 2026 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCL_EXEC__VISIBILITY_CONTROL_HPP_
#define RCL_EXEC__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RCL_EXEC_EXPORT __attribute__ ((dllexport))
    #define RCL_EXEC_IMPORT __attribute__ ((dllimport))
  #else
    #define RCL_EXEC_EXPORT __declspec(dllexport)
    #define RCL_EXEC_IMPORT __declspec(dllimport)
  #endif
  #ifdef RCL_EXEC_BUILDING_LIBRARY
    #define RCL_EXEC_PUBLIC RCL_EXEC_EXPORT
  #else
    #define RCL_EXEC_PUBLIC RCL_EXEC_IMPORT
  #endif
  #define RCL_EXEC_LOCAL
#else
  #define RCL_EXEC_EXPORT __attribute__ ((visibility("default")))
  #define RCL_EXEC_IMPORT
  #if __GNUC__ >= 4
    #define RCL_EXEC_PUBLIC __attribute__ ((visibility("default")))
    #define RCL_EXEC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RCL_EXEC_PUBLIC
    #define RCL_EXEC_LOCAL
  #endif
#endif

#endif  // RCL_EXEC__VISIBILITY_CONTROL_HPP_
