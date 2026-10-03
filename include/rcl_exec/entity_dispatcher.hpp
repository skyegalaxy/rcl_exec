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

#ifndef RCL_EXEC__ENTITY_DISPATCHER_HPP_
#define RCL_EXEC__ENTITY_DISPATCHER_HPP_

#include <memory>

#include "rcl_exec/types.hpp"

namespace rcl_exec
{

/// Base of the data a client library takes for one ready entity.
/**
 * The client library derives from this; its destructor is the deleter.
 * The engine only moves or destroys it, and may destroy it on any engine thread,
 * for example when a spin is cancelled between take() and execute().
 * A Python binding's derived destructor must acquire the GIL itself.
 */
class TakenPayload
{
public:
  virtual ~TakenPayload() = default;
};

/// Data produced by EntityDispatcher::take() and consumed by EntityDispatcher::execute().
/**
 * Null means nothing was taken.
 */
using TakenData = std::unique_ptr<TakenPayload>;

enum class ExecuteStatus
{
  Ok,
  Failed
};

/// The only upward call path from rcl_exec into a client library.
/**
 * Every method is called on an engine thread, possibly one that has never held the Python GIL.
 * Every method is noexcept: the implementation captures whatever user code throws or raises.
 */
class EntityDispatcher
{
public:
  virtual ~EntityDispatcher() = default;

  /// Take exactly one unit of data for a ready entity.
  /**
   * One message, request, response, timer call or waitable event.
   * \param[in] entity the ready entity.
   * \param[in] waitable_event the id the waitable passed to its ready callback; 0 for every
   *   other entity type.
   * \param[out] data the taken data; left null if there was nothing to take.
   * \return Failed if taking threw or raised.
   */
  virtual ExecuteStatus
  take(EntityHandle entity, int waitable_event, TakenData & data) noexcept = 0;

  /// Run the user callback with data previously returned by take().
  /**
   * The engine holds the callback group for the duration of the call.
   */
  virtual ExecuteStatus
  execute(EntityHandle entity, TakenData && data) noexcept = 0;
};

}  // namespace rcl_exec

#endif  // RCL_EXEC__ENTITY_DISPATCHER_HPP_
