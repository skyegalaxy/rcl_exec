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

#ifndef RCL_EXEC__DETAIL__READY_ENTITY_HPP_
#define RCL_EXEC__DETAIL__READY_ENTITY_HPP_

#include <functional>
#include <memory>
#include <utility>
#include <variant>

#include "rcl_exec/entity_dispatcher.hpp"
#include "rcl_exec/detail/scheduler.hpp"
#include "rcl_exec/detail/global_event_id_provider.hpp"

namespace rcl_exec
{
namespace detail
{
struct ReadyEntity
{
  struct ReadyTimerWithExecutedCallback
  {
    CBGScheduler::WeakEntityHandle timer_ptr;
        // must be called by the after executing the timer callback
    std::function<void()> timer_was_executed;

    bool expired() const
    {
      return timer_ptr.expired();
    }
  };

  std::variant<CBGScheduler::WeakEntityHandle, ReadyTimerWithExecutedCallback,
    CBGScheduler::WaitableWithEventType, CBGScheduler::CallbackEventType> entity;

  explicit ReadyEntity(const CBGScheduler::WeakEntityHandle & ptr)
  : entity(ptr), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const ReadyTimerWithExecutedCallback & timer)
  : entity(timer), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const CBGScheduler::WaitableWithEventType & ev)
  : entity(ev), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const CBGScheduler::CallbackEventType & ev)
  : entity(ev), id(GlobalEventIdProvider::get_next_id()) {}

  std::function<void()> get_execute_function(EntityDispatcher & dispatcher) const
  {
    return std::visit([&dispatcher](auto && entity) -> std::function<void()> {
               using T = std::decay_t<decltype(entity)>;
               if constexpr (std::is_same_v<T, CBGScheduler::WeakEntityHandle>) {
                 std::shared_ptr<const void> shr_ptr = entity.liveness.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }
                 return [shr_ptr = std::move(shr_ptr), handle = entity.handle, &dispatcher]() {
                          TakenData data;
                          if (dispatcher.take(handle, 0, data) != ExecuteStatus::Ok || !data) {
                            return;
                          }
                          dispatcher.execute(handle, std::move(data));
                        };
               } else if constexpr (std::is_same_v<T, ReadyTimerWithExecutedCallback>) {
                 auto shr_ptr = entity.timer_ptr.liveness.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }

                 return [shr_ptr = std::move(shr_ptr), handle = entity.timer_ptr.handle,
                        timer_executed_cb = entity.timer_was_executed, &dispatcher]() {
                          TakenData data;
                          if (dispatcher.take(handle, 0, data) != ExecuteStatus::Ok || !data) {
                              // timer was cancelled, skip it.
                            return;
                          }

                          if (dispatcher.execute(handle, std::move(data)) != ExecuteStatus::Ok) {
                            return;
                          }

                          // readd the timer to the timers manager
                          timer_executed_cb();
                        };
               } else if constexpr (std::is_same_v<T, CBGScheduler::WaitableWithEventType>) {
                 auto shr_ptr_in = entity.waitable.liveness.lock();
                 if (!shr_ptr_in) {
                   return std::function<void()>();
                 }

                 return [shr_ptr = std::move(shr_ptr_in), handle = entity.waitable.handle,
                        event_type = entity.internal_event_type, &dispatcher]() {
                          TakenData data;
                          if (dispatcher.take(handle, event_type, data) != ExecuteStatus::Ok ||
                          !data)
                          {
                            return;
                          }
                          dispatcher.execute(handle, std::move(data));
                        };
               } else if constexpr (std::is_same_v<T, CBGScheduler::CallbackEventType>) {
                 return entity.callback;
               }
        }, entity);
  }

  GlobalEventIdProvider::MonotonicId id;

    /**
     * Returns true if the event has expired / does not need to be executed any more
     */
  bool expired() const
  {
    return std::visit([](const auto & entity) {return entity.expired();}, entity);
  }
};
}  // namespace detail
}  // namespace rcl_exec

#endif  // RCL_EXEC__DETAIL__READY_ENTITY_HPP_
