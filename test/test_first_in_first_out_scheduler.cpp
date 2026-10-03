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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "rcl_exec/entity_dispatcher.hpp"
#include "rcl_exec/types.hpp"
#include "rcl_exec/detail/first_in_first_out_scheduler.hpp"
#include "rcl_exec/detail/global_event_id_provider.hpp"
#include "rcl_exec/detail/scheduler.hpp"

using rcl_exec::CallbackGroupType;
using rcl_exec::EntityHandle;
using rcl_exec::ExecuteStatus;
using rcl_exec::TakenData;
using rcl_exec::detail::CBGScheduler;
using rcl_exec::detail::FirstInFirstOutScheduler;
using rcl_exec::detail::GlobalEventIdProvider;
using rcl_exec::detail::Worker;

namespace
{

struct Call
{
  EntityHandle entity;
  int waitable_event;
};

class RecordingDispatcher : public rcl_exec::EntityDispatcher
{
public:
  ExecuteStatus
  take(EntityHandle entity, int waitable_event, TakenData & data) noexcept override
  {
    takes.push_back({entity, waitable_event});
    if (take_produces_data) {
      data = std::make_unique<rcl_exec::TakenPayload>();
    }
    return ExecuteStatus::Ok;
  }

  ExecuteStatus
  execute(EntityHandle entity, TakenData && data) noexcept override
  {
    executes.push_back({entity, 0});
    data.reset();
    return ExecuteStatus::Ok;
  }

  bool take_produces_data = true;
  std::vector<Call> takes;
  std::vector<Call> executes;
};

EntityHandle
subscription(std::uintptr_t address)
{
  return rcl_exec::make_entity_handle(reinterpret_cast<const rcl_subscription_t *>(address));
}

EntityHandle
timer(std::uintptr_t address)
{
  return rcl_exec::make_entity_handle(reinterpret_cast<const rcl_timer_t *>(address));
}

EntityHandle
waitable(std::uintptr_t address)
{
  return rcl_exec::make_waitable_handle(reinterpret_cast<const void *>(address));
}

class TestFirstInFirstOutScheduler : public ::testing::Test
{
protected:
  CBGScheduler::WeakEntityHandle
  weak(EntityHandle handle)
  {
    return {handle, alive_};
  }

  /// Pops the next ready entity, runs it and marks it executed. Returns false if none was ready.
  bool
  run_next()
  {
    auto ready = scheduler_.get_next_ready_entity();
    if (!ready.entity) {
      return false;
    }
    ready.entity->execute_function();
    scheduler_.mark_entity_as_executed(*ready.entity);
    return true;
  }

  RecordingDispatcher dispatcher_;
  int sync_calls_ = 0;
  FirstInFirstOutScheduler scheduler_{dispatcher_, [this]() {++sync_calls_;}};
  std::shared_ptr<int> alive_ = std::make_shared<int>(0);
};

}  // namespace

TEST_F(TestFirstInFirstOutScheduler, nothing_ready)
{
  scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  EXPECT_FALSE(run_next());
}

TEST_F(TestFirstInFirstOutScheduler, ready_entity_is_taken_and_executed_once_per_event)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(2);

  EXPECT_TRUE(run_next());
  EXPECT_TRUE(run_next());
  EXPECT_FALSE(run_next());
  ASSERT_EQ(2u, dispatcher_.executes.size());
  EXPECT_EQ(subscription(0x10), dispatcher_.takes[0].entity);
  EXPECT_EQ(0, dispatcher_.takes[0].waitable_event);
  EXPECT_EQ(subscription(0x10), dispatcher_.executes[1].entity);
}

TEST_F(TestFirstInFirstOutScheduler, entities_run_in_ready_order)
{
  auto * first_group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  auto * second_group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  second_group->get_ready_callback_for_entity(weak(subscription(0x20)))(1);
  first_group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);

  while (run_next()) {}
  ASSERT_EQ(2u, dispatcher_.executes.size());
  EXPECT_EQ(subscription(0x20), dispatcher_.executes[0].entity);
  EXPECT_EQ(subscription(0x10), dispatcher_.executes[1].entity);
}

TEST_F(TestFirstInFirstOutScheduler, expired_entity_is_skipped_without_take)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);
  alive_.reset();

  EXPECT_FALSE(run_next());
  EXPECT_TRUE(dispatcher_.takes.empty());
}

TEST_F(TestFirstInFirstOutScheduler, empty_take_skips_execute)
{
  dispatcher_.take_produces_data = false;
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);

  EXPECT_TRUE(run_next());
  EXPECT_EQ(1u, dispatcher_.takes.size());
  EXPECT_TRUE(dispatcher_.executes.empty());
}

TEST_F(TestFirstInFirstOutScheduler, mutually_exclusive_group_runs_one_at_a_time)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(2);

  auto first = scheduler_.get_next_ready_entity();
  ASSERT_TRUE(first.entity);
  EXPECT_FALSE(scheduler_.get_next_ready_entity().entity);

  scheduler_.mark_entity_as_executed(*first.entity);
  EXPECT_TRUE(scheduler_.get_next_ready_entity().entity);
}

TEST_F(TestFirstInFirstOutScheduler, reentrant_group_runs_concurrently)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::Reentrant);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(2);

  auto first = scheduler_.get_next_ready_entity();
  ASSERT_TRUE(first.entity);
  EXPECT_TRUE(first.moreEntitiesReady);
  EXPECT_TRUE(scheduler_.get_next_ready_entity().entity);
}

TEST_F(TestFirstInFirstOutScheduler, timer_is_rearmed_after_execute)
{
  int rearmed = 0;
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_timer(weak(timer(0x10)))([&rearmed]() {++rearmed;});

  EXPECT_TRUE(run_next());
  EXPECT_EQ(1u, dispatcher_.executes.size());
  EXPECT_EQ(1, rearmed);
}

TEST_F(TestFirstInFirstOutScheduler, cancelled_timer_is_not_rearmed)
{
  dispatcher_.take_produces_data = false;
  int rearmed = 0;
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_timer(weak(timer(0x10)))([&rearmed]() {++rearmed;});

  EXPECT_TRUE(run_next());
  EXPECT_TRUE(dispatcher_.executes.empty());
  EXPECT_EQ(0, rearmed);
}

TEST_F(TestFirstInFirstOutScheduler, waitable_event_id_reaches_take)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_waitable(weak(waitable(0x10)))(1, 7);

  EXPECT_TRUE(run_next());
  ASSERT_EQ(1u, dispatcher_.takes.size());
  EXPECT_EQ(waitable(0x10), dispatcher_.takes[0].entity);
  EXPECT_EQ(7, dispatcher_.takes[0].waitable_event);
}

TEST_F(TestFirstInFirstOutScheduler, callback_event_runs_its_callback)
{
  int called = 0;
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(CBGScheduler::CallbackEventType([&called]() {++called;}))(1);

  EXPECT_TRUE(run_next());
  EXPECT_EQ(1, called);
  EXPECT_TRUE(dispatcher_.takes.empty());
}

TEST_F(TestFirstInFirstOutScheduler, sync_runs_before_ready_entities)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);
  scheduler_.trigger_sync();

  EXPECT_TRUE(run_next());
  EXPECT_EQ(1, sync_calls_);
  EXPECT_TRUE(dispatcher_.executes.empty());
  EXPECT_TRUE(run_next());
  EXPECT_EQ(1u, dispatcher_.executes.size());
}

TEST_F(TestFirstInFirstOutScheduler, max_id_excludes_later_events)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::Reentrant);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);
  const auto max_id = GlobalEventIdProvider::get_last_id();
  group->get_ready_callback_for_entity(weak(subscription(0x20)))(1);

  auto ready = scheduler_.get_next_ready_entity(max_id);
  ASSERT_TRUE(ready.entity);
  ready.entity->execute_function();
  scheduler_.mark_entity_as_executed(*ready.entity);
  EXPECT_FALSE(scheduler_.get_next_ready_entity(max_id).entity);
  EXPECT_EQ(subscription(0x10), dispatcher_.executes.at(0).entity);
}

TEST_F(TestFirstInFirstOutScheduler, block_returns_immediately_when_work_is_pending)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);

  Worker worker;
  scheduler_.block_worker_thread(&worker);
  SUCCEED();
}

TEST_F(TestFirstInFirstOutScheduler, ready_entity_releases_blocked_worker)
{
  auto * group = scheduler_.add_callback_group(CallbackGroupType::MutuallyExclusive);
  Worker worker;
  std::thread blocked([this, &worker]() {scheduler_.block_worker_thread(&worker);});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  group->get_ready_callback_for_entity(weak(subscription(0x10)))(1);
  blocked.join();
  SUCCEED();
}

TEST_F(TestFirstInFirstOutScheduler, release_wakes_blocked_worker_and_keeps_it_from_blocking)
{
  Worker worker;
  std::thread blocked([this, &worker]() {scheduler_.block_worker_thread(&worker);});
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  scheduler_.release_all_worker_threads();
  blocked.join();

  Worker late;
  scheduler_.block_worker_thread(&late);
  SUCCEED();
}
