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
#include <thread>

#include "rcl_exec/detail/scheduler.hpp"

using rcl_exec::detail::Worker;
using rcl_exec::detail::WorkerQueue;
using namespace std::chrono_literals;

TEST(TestWorker, unblock_before_block_does_not_block)
{
  Worker worker;
  worker.prepare_block();
  worker.unblock();
  worker.block();
  EXPECT_TRUE(worker.wakeup);
}

TEST(TestWorker, block_for_times_out_without_unblock)
{
  Worker worker;
  worker.prepare_block();
  const auto start = std::chrono::steady_clock::now();
  worker.block_for(20ms);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 20ms);
  EXPECT_FALSE(worker.wakeup);
}

TEST(TestWorker, unblock_from_other_thread_wakes_block)
{
  Worker worker;
  worker.prepare_block();
  std::thread waker([&worker]() {
      std::this_thread::sleep_for(10ms);
      worker.unblock_thread_safe();
    });
  worker.block();
  waker.join();
  EXPECT_TRUE(worker.wakeup);
}

TEST(TestWorkerQueue, pop_on_empty_returns_null)
{
  WorkerQueue queue;
  EXPECT_EQ(nullptr, queue.pop_blocked_worker_thread());
}

TEST(TestWorkerQueue, pop_returns_most_recently_blocked)
{
  WorkerQueue queue;
  Worker first;
  Worker second;
  ASSERT_TRUE(queue.push_blocked_worker_thread(&first));
  ASSERT_TRUE(queue.push_blocked_worker_thread(&second));
  EXPECT_EQ(&second, queue.pop_blocked_worker_thread());
  EXPECT_EQ(&first, queue.pop_blocked_worker_thread());
  EXPECT_EQ(nullptr, queue.pop_blocked_worker_thread());
}

TEST(TestWorkerQueue, remove_worker_thread)
{
  WorkerQueue queue;
  Worker first;
  Worker second;
  queue.push_blocked_worker_thread(&first);
  queue.push_blocked_worker_thread(&second);
  queue.remove_worker_thread(&second);
  queue.remove_worker_thread(&second);
  EXPECT_EQ(&first, queue.pop_blocked_worker_thread());
  EXPECT_EQ(nullptr, queue.pop_blocked_worker_thread());
}

TEST(TestWorkerQueue, release_returns_all_and_rejects_later_pushes)
{
  WorkerQueue queue;
  Worker first;
  Worker second;
  queue.push_blocked_worker_thread(&first);
  queue.push_blocked_worker_thread(&second);
  const auto released = queue.release_all_worker_threads();
  EXPECT_EQ(2u, released.size());
  EXPECT_EQ(nullptr, queue.pop_blocked_worker_thread());
  EXPECT_FALSE(queue.push_blocked_worker_thread(&first));
}
