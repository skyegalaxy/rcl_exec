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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "rcl/context.h"
#include "rcl/error_handling.h"
#include "rcl/init.h"
#include "rcl/init_options.h"
#include "rcl/time.h"
#include "rcl_exec/clock.hpp"
#include "rcutils/allocator.h"
#include "rcutils/time.h"

namespace
{

using namespace std::chrono_literals;

void init_context(rcl_context_t & context)
{
  context = rcl_get_zero_initialized_context();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ASSERT_EQ(RCL_RET_OK, rcl_init_options_init(&init_options, rcutils_get_default_allocator()));
  ASSERT_EQ(RCL_RET_OK, rcl_init(0, nullptr, &init_options, &context));
  ASSERT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options));
}

void fini_context(rcl_context_t & context)
{
  if (rcl_context_is_valid(&context)) {
    EXPECT_EQ(RCL_RET_OK, rcl_shutdown(&context));
  }
  EXPECT_EQ(RCL_RET_OK, rcl_context_fini(&context));
}

// What rclcpp::TimeSource does on a use_sim_time change.
void set_ros_time_override(rcl_exec::Clock & clock, bool enabled)
{
  std::lock_guard<std::mutex> clock_guard(clock.get_clock_mutex());
  if (enabled) {
    ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(clock.get_clock_handle()));
  } else {
    ASSERT_EQ(RCL_RET_OK, rcl_disable_ros_time_override(clock.get_clock_handle()));
  }
}

// What rclcpp's context on-shutdown hook does.
void shutdown(rcl_context_t & context, rcl_exec::Clock & clock)
{
  ASSERT_EQ(RCL_RET_OK, rcl_shutdown(&context));
  clock.notify_shutdown();
}

}  // namespace


TEST(TestClock, clock_type_access) {
  rcl_exec::Clock ros_clock(RCL_ROS_TIME);
  EXPECT_EQ(RCL_ROS_TIME, ros_clock.get_clock_type());

  rcl_exec::Clock system_clock(RCL_SYSTEM_TIME);
  EXPECT_EQ(RCL_SYSTEM_TIME, system_clock.get_clock_type());

  rcl_exec::Clock steady_clock(RCL_STEADY_TIME);
  EXPECT_EQ(RCL_STEADY_TIME, steady_clock.get_clock_type());
}

// Check that the clock may go out of the scope before the jump callback without leading in UB.
TEST(TestClock, clock_jump_callback_destruction_order) {
  rcl_exec::JumpHandler::SharedPtr handler;
  {
    rcl_exec::Clock ros_clock(RCL_ROS_TIME);
    rcl_jump_threshold_t threshold;
    threshold.on_clock_change = false;
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 1;
    ASSERT_EQ(
      RCL_RET_OK,
      ros_clock.create_jump_callback(
        []() {}, [](const rcl_time_jump_t &) {}, threshold, handler));
  }
}

TEST(TestClock, time_sources) {
  rcl_exec::Clock ros_clock(RCL_ROS_TIME);
  EXPECT_NE(0, ros_clock.now());

  rcl_exec::Clock system_clock(RCL_SYSTEM_TIME);
  EXPECT_NE(0, system_clock.now());

  rcl_exec::Clock steady_clock(RCL_STEADY_TIME);
  EXPECT_NE(0, steady_clock.now());
}

class TestClockSleep : public ::testing::Test
{
protected:
  void SetUp()
  {
    init_context(context);
  }

  void TearDown()
  {
    fini_context(context);
  }

  rcl_context_t context;
};

TEST_F(TestClockSleep, sleep_until_invalid_context) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto until = clock.now();

  EXPECT_FALSE(clock.sleep_until(until, nullptr));

  rcl_context_t uninitialized_context = rcl_get_zero_initialized_context();
  EXPECT_FALSE(clock.sleep_until(until, &uninitialized_context));

  rcl_context_t shutdown_context;
  init_context(shutdown_context);
  ASSERT_EQ(RCL_RET_OK, rcl_shutdown(&shutdown_context));
  EXPECT_FALSE(clock.sleep_until(until, &shutdown_context));
  fini_context(shutdown_context);
}

TEST_F(TestClockSleep, sleep_until_non_global_context) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto until = clock.now() + 1;

  rcl_context_t non_global_context;
  init_context(non_global_context);
  EXPECT_TRUE(clock.sleep_until(until, &non_global_context));
  fini_context(non_global_context);
}

TEST_F(TestClockSleep, sleep_until_basic_system) {
  const auto milliseconds = 300;
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto delay = RCUTILS_MS_TO_NS(milliseconds);
  auto sleep_until = clock.now() + delay;

  auto start = std::chrono::system_clock::now();
  ASSERT_TRUE(clock.sleep_until(sleep_until, &context));
  auto end = std::chrono::system_clock::now();

  EXPECT_GE(clock.now(), sleep_until);
  EXPECT_GE(end - start, std::chrono::milliseconds(milliseconds));
}

TEST_F(TestClockSleep, sleep_until_basic_steady) {
  const auto milliseconds = 300;
  rcl_exec::Clock clock(RCL_STEADY_TIME);
  auto delay = RCUTILS_MS_TO_NS(milliseconds);
  auto sleep_until = clock.now() + delay;

  auto steady_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(clock.sleep_until(sleep_until, &context));
  auto steady_end = std::chrono::steady_clock::now();

  EXPECT_GE(clock.now(), sleep_until);
  EXPECT_GE(steady_end - steady_start, std::chrono::milliseconds(milliseconds));
}

TEST_F(TestClockSleep, sleep_until_steady_past_returns_immediately) {
  rcl_exec::Clock clock(RCL_STEADY_TIME);
  auto until = clock.now() - RCUTILS_S_TO_NS(1000);
  // This should return immediately, other possible behavior might be sleep forever and timeout
  ASSERT_TRUE(clock.sleep_until(until, &context));
}

TEST_F(TestClockSleep, sleep_until_system_past_returns_immediately) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto until = clock.now() - RCUTILS_S_TO_NS(1000);
  // This should return immediately, other possible behavior might be sleep forever and timeout
  ASSERT_TRUE(clock.sleep_until(until, &context));
}

TEST_F(TestClockSleep, sleep_until_ros_time_enable_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);

  // 5 second timeout, but it should be interrupted right away
  const auto until = clock->now() + RCUTILS_S_TO_NS(5);

  // Try sleeping with ROS time off, then turn it on to interrupt
  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, until, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_until(until, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  set_ros_time_override(*clock, true);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_until_ros_time_disable_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);
  set_ros_time_override(*clock, true);

  // /clock shouldn't be publishing, shouldn't be possible to reach timeout
  const auto until = clock->now() + RCUTILS_S_TO_NS(600);

  // Try sleeping with ROS time off, then turn it on to interrupt
  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, until, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_until(until, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  set_ros_time_override(*clock, false);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_until_shutdown_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);
  set_ros_time_override(*clock, true);

  // the timeout doesn't matter here - no /clock is being published, so it should never wake
  const auto until = clock->now() + RCUTILS_S_TO_NS(600);

  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, until, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_until(until, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  shutdown(context, *clock);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_until_basic_ros) {
  rcl_exec::Clock clock(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock.get_clock_handle();

  ASSERT_EQ(RCL_ROS_TIME, clock.get_clock_type());

  // Not zero, because 0 means time not initialized
  const rcl_time_point_value_t start_time = 1337;
  const rcl_time_point_value_t end_time = start_time + 1;

  // Initialize time
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, start_time));

  const auto until = end_time;

  bool sleep_succeeded = false;
  auto sleep_thread = std::thread(
    [this, &clock, until, &sleep_succeeded]() {
      sleep_succeeded = clock.sleep_until(until, &context);
    });

  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // False because still sleeping
  EXPECT_FALSE(sleep_succeeded);

  // Jump time to the end
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, end_time));
  ASSERT_EQ(until, clock.now());

  sleep_thread.join();
  EXPECT_TRUE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_for_invalid_context) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto rel_time = RCUTILS_S_TO_NS(1);

  EXPECT_FALSE(clock.sleep_for(rel_time, nullptr));

  rcl_context_t uninitialized_context = rcl_get_zero_initialized_context();
  EXPECT_FALSE(clock.sleep_for(rel_time, &uninitialized_context));

  rcl_context_t shutdown_context;
  init_context(shutdown_context);
  ASSERT_EQ(RCL_RET_OK, rcl_shutdown(&shutdown_context));
  EXPECT_FALSE(clock.sleep_for(rel_time, &shutdown_context));
  fini_context(shutdown_context);
}

TEST_F(TestClockSleep, sleep_for_non_global_context) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto rel_time = 1;

  rcl_context_t non_global_context;
  init_context(non_global_context);
  EXPECT_TRUE(clock.sleep_for(rel_time, &non_global_context));
  fini_context(non_global_context);
}

TEST_F(TestClockSleep, sleep_for_basic_system) {
  const auto milliseconds = 300;
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto rel_time = RCUTILS_MS_TO_NS(milliseconds);

  auto start = std::chrono::system_clock::now();
  ASSERT_TRUE(clock.sleep_for(rel_time, &context));
  auto end = std::chrono::system_clock::now();

  EXPECT_GE(end - start, std::chrono::milliseconds(milliseconds));
}

TEST_F(TestClockSleep, sleep_for_basic_steady) {
  const auto milliseconds = 300;
  rcl_exec::Clock clock(RCL_STEADY_TIME);
  auto rel_time = RCUTILS_MS_TO_NS(milliseconds);

  auto steady_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(clock.sleep_for(rel_time, &context));
  auto steady_end = std::chrono::steady_clock::now();

  EXPECT_GE(steady_end - steady_start, std::chrono::milliseconds(milliseconds));
}

TEST_F(TestClockSleep, sleep_for_steady_past_returns_immediately) {
  rcl_exec::Clock clock(RCL_STEADY_TIME);
  auto rel_time = -RCUTILS_S_TO_NS(1000);
  // This should return immediately
  ASSERT_TRUE(clock.sleep_for(rel_time, &context));
}

TEST_F(TestClockSleep, sleep_for_system_past_returns_immediately) {
  rcl_exec::Clock clock(RCL_SYSTEM_TIME);
  auto rel_time = -RCUTILS_S_TO_NS(1000);
  // This should return immediately
  ASSERT_TRUE(clock.sleep_for(rel_time, &context));
}

TEST_F(TestClockSleep, sleep_for_ros_time_enable_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);

  // 5 second timeout, but it should be interrupted right away
  const auto rel_time = RCUTILS_S_TO_NS(5);

  // Try sleeping with ROS time off, then turn it on to interrupt
  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, rel_time, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_for(rel_time, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  set_ros_time_override(*clock, true);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_for_ros_time_disable_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);
  set_ros_time_override(*clock, true);

  // /clock shouldn't be publishing, shouldn't be possible to reach timeout
  const auto rel_time = RCUTILS_S_TO_NS(600);

  // Try sleeping with ROS time off, then turn it on to interrupt
  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, rel_time, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_for(rel_time, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  set_ros_time_override(*clock, false);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_for_shutdown_interrupt) {
  auto clock = std::make_shared<rcl_exec::Clock>(RCL_ROS_TIME);
  set_ros_time_override(*clock, true);

  // the timeout doesn't matter here - no /clock is being published, so it should never wake
  const auto rel_time = RCUTILS_S_TO_NS(600);

  bool sleep_succeeded = true;
  auto sleep_thread = std::thread(
    [this, clock, rel_time, &sleep_succeeded]() {
      sleep_succeeded = clock->sleep_for(rel_time, &context);
    });
  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  shutdown(context, *clock);
  sleep_thread.join();
  EXPECT_FALSE(sleep_succeeded);
}

TEST_F(TestClockSleep, sleep_for_basic_ros) {
  rcl_exec::Clock clock(RCL_ROS_TIME);
  rcl_clock_t * rcl_clock = clock.get_clock_handle();

  ASSERT_EQ(RCL_ROS_TIME, clock.get_clock_type());

  // Not zero, because 0 means time not initialized
  const rcl_time_point_value_t start_time = 1337;
  const rcl_time_point_value_t end_time = start_time + 1;

  // Initialize time
  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(rcl_clock));
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, start_time));

  const auto rel_time = 1;

  bool sleep_succeeded = false;
  auto sleep_thread = std::thread(
    [this, &clock, rel_time, &sleep_succeeded]() {
      sleep_succeeded = clock.sleep_for(rel_time, &context);
    });

  // yield execution long enough to let the sleep thread get to waiting on the condition variable
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // False because still sleeping
  EXPECT_FALSE(sleep_succeeded);

  // Jump time to the end
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(rcl_clock, end_time));
  ASSERT_EQ(end_time, clock.now());

  sleep_thread.join();
  EXPECT_TRUE(sleep_succeeded);
}

class TestClockStarted : public ::testing::Test
{
protected:
  void SetUp()
  {
    init_context(context);
  }

  void TearDown()
  {
    fini_context(context);
  }

  rcl_context_t context;
};

TEST_F(TestClockStarted, started_timeout) {
  rcl_exec::Clock ros_clock(RCL_ROS_TIME);
  auto ros_clock_handle = ros_clock.get_clock_handle();

  EXPECT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(ros_clock_handle));
  bool ros_time_active = false;
  EXPECT_EQ(RCL_RET_OK, ros_clock.ros_time_is_active(ros_time_active));
  EXPECT_TRUE(ros_time_active);

  EXPECT_EQ(RCL_RET_OK, rcl_set_ros_time_override(ros_clock_handle, 0));

  EXPECT_FALSE(ros_clock.started());
  EXPECT_FALSE(ros_clock.wait_until_started(RCUTILS_MS_TO_NS(10), &context));

  std::thread t([this, &ros_clock]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      shutdown(context, ros_clock);
    });

  // Test shutdown escape hatch (otherwise this waits indefinitely)
  EXPECT_FALSE(ros_clock.wait_until_started(&context));
  t.join();
}
