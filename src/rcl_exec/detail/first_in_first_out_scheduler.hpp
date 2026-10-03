// Copyright 2024 Cellumation GmbH.
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

#ifndef RCL_EXEC__DETAIL__FIRST_IN_FIRST_OUT_SCHEDULER_HPP_
#define RCL_EXEC__DETAIL__FIRST_IN_FIRST_OUT_SCHEDULER_HPP_

#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "rcl_exec/entity_dispatcher.hpp"
#include "rcl_exec/types.hpp"
#include "rcl_exec/detail/ready_entity.hpp"
#include "rcl_exec/detail/scheduler.hpp"
#include "rcl_exec/detail/global_event_id_provider.hpp"

namespace rcl_exec
{
namespace detail
{
struct FirstInFirstOutCallbackGroupHandle final : public CBGScheduler::CallbackGroupHandle
{
public:
  FirstInFirstOutCallbackGroupHandle(
    CBGScheduler & scheduler, CallbackGroupType type, EntityDispatcher & dispatcher)
  : CallbackGroupHandle(scheduler, type), dispatcher(dispatcher)
  {
  }

  std::function<void(size_t)> get_ready_callback_for_entity(
    const CBGScheduler::WeakEntityHandle & entity) final;
  std::function<void(std::function<void()> executed_callback)> get_ready_callback_for_timer(
    const CBGScheduler::WeakEntityHandle & timer) final;
  std::function<void(size_t,
    int)> get_ready_callback_for_waitable(const CBGScheduler::WeakEntityHandle & waitable) final;
  std::function<void(size_t)> get_ready_callback_for_entity(
    const CBGScheduler::CallbackEventType & entity) final;

  std::optional<CBGScheduler::ExecutableEntity> get_next_ready_entity();
  std::optional<CBGScheduler::ExecutableEntity> get_next_ready_entity(
    GlobalEventIdProvider::MonotonicId max_id);

  bool has_ready_entities() const final
  {
    return !ready_entities.empty();
  }

private:
  EntityDispatcher & dispatcher;
  std::deque<ReadyEntity> ready_entities;
};

class FirstInFirstOutScheduler : public CBGScheduler
{
public:
  FirstInFirstOutScheduler(EntityDispatcher & dispatcher, std::function<void()> sync_function)
  : CBGScheduler(std::move(sync_function)), dispatcher(dispatcher)
  {
  }

private:
  ExecutableEntityWithInfo get_next_ready_entity_intern() final;
  ExecutableEntityWithInfo get_next_ready_entity_intern(
    GlobalEventIdProvider::MonotonicId max_id) final;

  std::unique_ptr<CallbackGroupHandle> get_handle_for_callback_group(
    CallbackGroupType callback_group_type) final;

  EntityDispatcher & dispatcher;

  std::vector<std::unique_ptr<FirstInFirstOutCallbackGroupHandle>> callback_group_handles;
};
}  // namespace detail
}  // namespace rcl_exec

#endif  // RCL_EXEC__DETAIL__FIRST_IN_FIRST_OUT_SCHEDULER_HPP_
