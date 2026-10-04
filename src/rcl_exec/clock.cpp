// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rcl_exec/clock.hpp"

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl/error_handling.h"
#include "rcutils/logging_macros.h"

namespace rcl_exec
{

class Clock::Impl
{
public:
  explicit Impl(rcl_clock_type_t clock_type)
  : allocator_{rcl_get_default_allocator()}
  {
    rcl_ret_t ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
    if (ret == RCL_RET_BAD_ALLOC) {
      rcl_reset_error();
      throw std::bad_alloc{};
    }
    if (ret != RCL_RET_OK) {
      std::string message = std::string("failed to initialize rcl clock: ") +
        rcl_get_error_string().str;
      rcl_reset_error();
      throw std::invalid_argument(message);
    }
  }

  ~Impl()
  {
    rcl_ret_t ret = rcl_clock_fini(&rcl_clock_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
    }
  }

  rcl_clock_t rcl_clock_;
  rcl_allocator_t allocator_;
  bool shutdown_ = false;
  std::condition_variable cv_;
  std::mutex wait_mutex_;
  std::mutex clock_mutex_;
};

JumpHandler::JumpHandler(
  pre_callback_t pre_callback,
  post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
: pre_callback(std::move(pre_callback)),
  post_callback(std::move(post_callback)),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
: impl_(new Clock::Impl(clock_type)) {}

Clock::~Clock() {}

rcl_time_point_value_t
Clock::now() const noexcept
{
  rcl_time_point_value_t now = 0;

  auto ret = rcl_clock_get_now(&impl_->rcl_clock_, &now);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("could not get current time stamp: %s", rcl_get_error_string().str);
    rcl_reset_error();
    return 0;
  }

  return now;
}

bool
Clock::sleep_until(rcl_time_point_value_t until, rcl_context_t * context)
{
  if (!context || !rcl_context_is_valid(context)) {
    return false;
  }
  const auto this_clock_type = get_clock_type();
  bool time_source_changed = false;

  if (this_clock_type == RCL_STEADY_TIME) {
    // Synchronize because RCL steady clock epoch might differ from chrono::steady_clock epoch
    const rcl_time_point_value_t rcl_entry = now();
    const std::chrono::steady_clock::time_point chrono_entry = std::chrono::steady_clock::now();
    const rcl_duration_value_t delta_t = until - rcl_entry;
    const std::chrono::steady_clock::time_point chrono_until =
      chrono_entry + std::chrono::nanoseconds(delta_t);

    // loop over spurious wakeups but notice shutdown
    std::unique_lock lock(impl_->wait_mutex_);
    while (now() < until && !impl_->shutdown_ && rcl_context_is_valid(context)) {
      impl_->cv_.wait_until(lock, chrono_until);
    }
  } else if (this_clock_type == RCL_SYSTEM_TIME) {
    auto system_time = std::chrono::system_clock::time_point(
      // Cast because system clock resolution is too big for nanoseconds on some systems
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::nanoseconds(until)));

    // loop over spurious wakeups but notice shutdown
    std::unique_lock lock(impl_->wait_mutex_);
    while (now() < until && !impl_->shutdown_ && rcl_context_is_valid(context)) {
      impl_->cv_.wait_until(lock, system_time);
    }
  } else if (this_clock_type == RCL_ROS_TIME) {
    // Install jump handler for any amount of time change, for two purposes:
    // - if ROS time is active, check if time reached on each new clock sample
    // - Trigger via on_clock_change to detect if time source changes, to invalidate sleep
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    // 0 is disable, so -1 and 1 are smallest possible time changes
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    JumpHandler::SharedPtr clock_handler;
    rcl_ret_t ret = create_jump_callback(
      nullptr,
      [this, &time_source_changed](const rcl_time_jump_t & jump) {
        if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
          std::lock_guard<std::mutex> lk(impl_->wait_mutex_);
          time_source_changed = true;
        }
        impl_->cv_.notify_one();
      },
      threshold, clock_handler);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to add time jump callback: %s", rcl_get_error_string().str);
      rcl_reset_error();
      return false;
    }

    bool ros_time_active = false;
    if (ros_time_is_active(ros_time_active) != RCL_RET_OK) {
      rcl_reset_error();
    }
    if (!ros_time_active) {
      auto system_time = std::chrono::system_clock::time_point(
        // Cast because system clock resolution is too big for nanoseconds on some systems
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::nanoseconds(until)));

      // loop over spurious wakeups but notice shutdown or time source change
      std::unique_lock lock(impl_->wait_mutex_);
      while (now() < until && !impl_->shutdown_ && rcl_context_is_valid(context) &&
        !time_source_changed)
      {
        impl_->cv_.wait_until(lock, system_time);
      }
    } else {
      // RCL_ROS_TIME with ros_time_is_active.
      // Just wait without "until" because installed
      // jump callbacks wake the cv on every new sample.
      std::unique_lock lock(impl_->wait_mutex_);
      while (now() < until && !impl_->shutdown_ && rcl_context_is_valid(context) &&
        !time_source_changed)
      {
        impl_->cv_.wait(lock);
      }
    }
  }

  if (!rcl_context_is_valid(context) || time_source_changed) {
    return false;
  }

  return now() >= until;
}

bool
Clock::sleep_for(rcl_duration_value_t rel_time, rcl_context_t * context)
{
  return sleep_until(now() + rel_time, context);
}

void
Clock::notify_shutdown()
{
  {
    std::unique_lock lock(impl_->wait_mutex_);
    impl_->shutdown_ = true;
  }
  impl_->cv_.notify_one();
}

bool
Clock::started()
{
  return rcl_clock_time_started(get_clock_handle());
}

bool
Clock::wait_until_started(rcl_context_t * context)
{
  if (!context || !rcl_context_is_valid(context)) {
    return false;
  }

  if (started()) {
    return true;
  } else {
    // Wait until the first non-zero time
    return sleep_until(1, context);
  }
}

bool
Clock::wait_until_started(
  rcl_duration_value_t timeout,
  rcl_context_t * context,
  rcl_duration_value_t wait_tick_ns)
{
  if (!context || !rcl_context_is_valid(context)) {
    return false;
  }

  Clock timeout_clock = Clock(RCL_STEADY_TIME);
  rcl_time_point_value_t start = timeout_clock.now();

  // Check if the clock has started every wait_tick_ns nanoseconds
  // Context check checks for shutdown
  while (!started() && rcl_context_is_valid(context)) {
    if (timeout < wait_tick_ns) {
      timeout_clock.sleep_for(timeout, context);
    } else {
      rcl_duration_value_t time_left = start + timeout - timeout_clock.now();
      if (time_left > wait_tick_ns) {
        timeout_clock.sleep_for(wait_tick_ns, context);
      } else {
        timeout_clock.sleep_for(time_left, context);
      }
    }

    if (timeout_clock.now() - start > timeout) {
      return started();
    }
  }
  return started();
}


rcl_ret_t
Clock::ros_time_is_active(bool & active)
{
  active = false;
  if (!rcl_clock_valid(&impl_->rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return RCL_RET_OK;
  }

  return rcl_is_enabled_ros_time_override(&impl_->rcl_clock_, &active);
}

rcl_clock_t *
Clock::get_clock_handle() noexcept
{
  return &impl_->rcl_clock_;
}

rcl_clock_type_t
Clock::get_clock_type() const noexcept
{
  return impl_->rcl_clock_.type;
}

std::mutex &
Clock::get_clock_mutex() noexcept
{
  return impl_->clock_mutex_;
}

void
Clock::on_time_jump(
  const rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  const auto * handler = static_cast<JumpHandler *>(user_data);
  if (nullptr == handler) {
    return;
  }
  if (before_jump && handler->pre_callback) {
    handler->pre_callback();
  } else if (!before_jump && handler->post_callback) {
    handler->post_callback(*time_jump);
  }
}

rcl_ret_t
Clock::create_jump_callback(
  const JumpHandler::pre_callback_t & pre_callback,
  const JumpHandler::post_callback_t & post_callback,
  const rcl_jump_threshold_t & threshold,
  JumpHandler::SharedPtr & jump_handler)
{
  // Allocate a new jump handler
  JumpHandler::UniquePtr handler(new JumpHandler(pre_callback, post_callback, threshold));
  if (nullptr == handler) {
    throw std::bad_alloc{};
  }

  {
    std::lock_guard<std::mutex> clock_guard(impl_->clock_mutex_);
    // Try to add the jump callback to the clock
    rcl_ret_t ret = rcl_clock_add_jump_callback(
      &impl_->rcl_clock_, threshold, Clock::on_time_jump,
      handler.get());
    if (RCL_RET_OK != ret) {
      return ret;
    }
  }

  std::weak_ptr<Clock::Impl> weak_impl = impl_;
  // *INDENT-OFF*
  // create shared_ptr that removes the callback automatically when all copies are destructed
  jump_handler = JumpHandler::SharedPtr(handler.release(),
    [weak_impl](JumpHandler * handler) noexcept {
    auto shared_impl = weak_impl.lock();
    if (shared_impl) {
      std::lock_guard<std::mutex> clock_guard(shared_impl->clock_mutex_);
      rcl_ret_t ret = rcl_clock_remove_jump_callback(&shared_impl->rcl_clock_,
          Clock::on_time_jump, handler);
      if (RCL_RET_OK != ret) {
        RCUTILS_LOG_ERROR("Failed to remove time jump callback");
      }
    }
    delete handler;
  });
  // *INDENT-ON*
  return RCL_RET_OK;
}

class ClockWaiter::ClockWaiterImpl
{
private:
  std::condition_variable cv_;

  Clock::SharedPtr clock_;
  bool time_source_changed_ = false;
  std::function<void(const rcl_time_jump_t &)> post_time_jump_callback;

  bool
  wait_until_system_time(
    std::unique_lock<std::mutex> & lock,
    rcl_time_point_value_t abs_time, const std::function<bool ()> & pred)
  {
    auto system_time = std::chrono::system_clock::time_point(
    // Cast because system clock resolution is too big for nanoseconds on some systems
    std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::nanoseconds(abs_time)));

    return cv_.wait_until(lock, system_time, pred);
  }

  bool
  wait_until_steady_time(
    std::unique_lock<std::mutex> & lock,
    rcl_time_point_value_t abs_time, const std::function<bool ()> & pred)
  {
    // Synchronize because RCL steady clock epoch might differ from chrono::steady_clock epoch
    const rcl_time_point_value_t rcl_entry = clock_->now();
    const std::chrono::steady_clock::time_point chrono_entry = std::chrono::steady_clock::now();
    const rcl_duration_value_t delta_t = abs_time - rcl_entry;
    const std::chrono::steady_clock::time_point chrono_until =
      chrono_entry + std::chrono::nanoseconds(delta_t);

    return cv_.wait_until(lock, chrono_until, pred);
  }


  bool
  wait_until_ros_time(
    std::unique_lock<std::mutex> & lock,
    rcl_time_point_value_t abs_time, const std::function<bool ()> & pred)
  {
    // Install jump handler for any amount of time change, for two purposes:
    // - if ROS time is active, check if time reached on each new clock sample
    // - Trigger via on_clock_change to detect if time source changes, to invalidate sleep
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = true;
    // 0 is disable, so -1 and 1 are smallest possible time changes
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;

    time_source_changed_ = false;

    post_time_jump_callback = [this, &lock] (const rcl_time_jump_t & jump)
      {
        if (jump.clock_change != RCL_ROS_TIME_NO_CHANGE) {
          std::lock_guard<std::mutex> lk(*lock.mutex());
          time_source_changed_ = true;
        }
        cv_.notify_one();
      };

    // Note this is a trade-off. Adding the callback for every call
    // is expensive for high frequency calls. For low frequency waits
    // its more overhead to have the callback being called all the time.
    // As we expect the use case to be low frequency calls to wait_until
    // with relative big pauses between the calls, we install it on demand.
    JumpHandler::SharedPtr clock_handler;
    if (clock_->create_jump_callback(
        nullptr,
        post_time_jump_callback,
        threshold, clock_handler) != RCL_RET_OK)
    {
      RCUTILS_LOG_ERROR("Failed to add time jump callback: %s", rcl_get_error_string().str);
      rcl_reset_error();
      return false;
    }

    bool ros_time_active = false;
    if (clock_->ros_time_is_active(ros_time_active) != RCL_RET_OK) {
      rcl_reset_error();
    }
    if (!ros_time_active) {
      auto system_time = std::chrono::system_clock::time_point(
        // Cast because system clock resolution is too big for nanoseconds on some systems
        std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::nanoseconds(abs_time)));

      return cv_.wait_until(lock, system_time, [this, &pred] () {
                 return time_source_changed_ || pred();
      });
    }

    // RCL_ROS_TIME with ros_time_is_active.
    // Just wait without "until" because installed
    // jump callbacks wake the cv on every new sample.
    cv_.wait(lock, [this, &pred, &abs_time] () {
        return clock_->now() >= abs_time || time_source_changed_ || pred();
    });

    return clock_->now() < abs_time;
  }

public:
  explicit ClockWaiterImpl(const Clock::SharedPtr & clock)
  :clock_(clock)
  {
  }

  bool
  wait_until(
    std::unique_lock<std::mutex> & lock,
    rcl_time_point_value_t abs_time, const std::function<bool ()> & pred)
  {
    switch(clock_->get_clock_type()) {
      case RCL_CLOCK_UNINITIALIZED:
        return false;
      case RCL_ROS_TIME:
        return wait_until_ros_time(lock, abs_time, pred);
        break;
      case RCL_STEADY_TIME:
        return wait_until_steady_time(lock, abs_time, pred);
        break;
      case RCL_SYSTEM_TIME:
        return wait_until_system_time(lock, abs_time, pred);
        break;
    }

    return false;
  }

  void
  notify_one()
  {
    cv_.notify_one();
  }
};

ClockWaiter::ClockWaiter(const Clock::SharedPtr & clock)
:impl_(std::make_unique<ClockWaiterImpl>(clock))
{
}

ClockWaiter::~ClockWaiter() = default;

bool
ClockWaiter::wait_until(
  std::unique_lock<std::mutex> & lock,
  rcl_time_point_value_t abs_time, const std::function<bool ()> & pred)
{
  return impl_->wait_until(lock, abs_time, pred);
}

void
ClockWaiter::notify_one()
{
  impl_->notify_one();
}

class ClockConditionalVariable::Impl
{
  std::mutex pred_mutex_;
  bool shutdown_ = false;
  ClockWaiter::UniquePtr clock_;

public:
  explicit Impl(const Clock::SharedPtr & clock)
  : clock_(std::make_unique<ClockWaiter>(clock))
  {
  }

  bool
  wait_until(
    std::unique_lock<std::mutex> & lock, rcl_time_point_value_t until,
    const std::function<bool ()> & pred)
  {
    if(lock.mutex() != &pred_mutex_) {
      throw std::runtime_error(
          "ClockConditionalVariable::wait_until: Internal error, given lock does not use"
          " mutex returned by this->mutex()");
    }

    clock_->wait_until(lock, until, [this, &pred] () -> bool {
        return shutdown_ || pred();
      });
    return true;
  }

  void
  notify_one()
  {
    clock_->notify_one();
  }

  void
  notify_shutdown()
  {
    {
      std::unique_lock lock(pred_mutex_);
      shutdown_ = true;
    }
    clock_->notify_one();
  }

  std::mutex &
  mutex()
  {
    return pred_mutex_;
  }
};

ClockConditionalVariable::ClockConditionalVariable(const Clock::SharedPtr & clock)
:impl_(std::make_unique<Impl>(clock))
{
}

ClockConditionalVariable::~ClockConditionalVariable() = default;

void
ClockConditionalVariable::notify_one()
{
  impl_->notify_one();
}

void
ClockConditionalVariable::notify_shutdown()
{
  impl_->notify_shutdown();
}

bool
ClockConditionalVariable::wait_until(
  std::unique_lock<std::mutex> & lock, rcl_time_point_value_t until,
  const std::function<bool ()> & pred)
{
  return impl_->wait_until(lock, until, pred);
}

std::mutex &
ClockConditionalVariable::mutex()
{
  return impl_->mutex();
}

}  // namespace rcl_exec
