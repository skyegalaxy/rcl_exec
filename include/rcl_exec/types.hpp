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

#ifndef RCL_EXEC__TYPES_HPP_
#define RCL_EXEC__TYPES_HPP_

#include <cstdint>

#include "rcl/client.h"
#include "rcl/guard_condition.h"
#include "rcl/service.h"
#include "rcl/subscription.h"
#include "rcl/timer.h"

namespace rcl_exec
{

/// The type of entity an EntityHandle refers to.
enum class EntityType
{
  Subscription,
  Timer,
  Service,
  Client,
  GuardCondition,
  Waitable
};

/// Identifies an entity by its rcl handle. Never by a client-library object.
/**
 * The engine uses the handle as a key and, for rcl-native types, to register rcl callbacks.
 * It never dereferences a waitable key.
 */
struct EntityHandle
{
  EntityType type;
  const void * handle;

  friend constexpr bool operator==(const EntityHandle &, const EntityHandle &) = default;
};

constexpr EntityHandle
make_entity_handle(const rcl_subscription_t * handle) noexcept
{
  return {EntityType::Subscription, handle};
}

constexpr EntityHandle
make_entity_handle(const rcl_timer_t * handle) noexcept
{
  return {EntityType::Timer, handle};
}

constexpr EntityHandle
make_entity_handle(const rcl_service_t * handle) noexcept
{
  return {EntityType::Service, handle};
}

constexpr EntityHandle
make_entity_handle(const rcl_client_t * handle) noexcept
{
  return {EntityType::Client, handle};
}

constexpr EntityHandle
make_entity_handle(const rcl_guard_condition_t * handle) noexcept
{
  return {EntityType::GuardCondition, handle};
}

constexpr EntityHandle
make_waitable_handle(const void * key) noexcept
{
  return {EntityType::Waitable, key};
}

/// Opaque callback-group identifier; the client library maps it back to its own group object.
enum class CallbackGroupId : std::uint64_t {};

/// Mutual-exclusion semantics of a callback group.
enum class CallbackGroupType
{
  /// At most one executable of the group runs at a time.
  MutuallyExclusive,
  /// Executables of the group may run concurrently.
  Reentrant
};

}  // namespace rcl_exec

#endif  // RCL_EXEC__TYPES_HPP_
