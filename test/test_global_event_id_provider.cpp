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

#include <algorithm>
#include <cstdint>
#include <thread>
#include <vector>

#include "rcl_exec/detail/global_event_id_provider.hpp"

using rcl_exec::detail::GlobalEventIdProvider;

TEST(TestGlobalEventIdProvider, next_id_increments_and_becomes_last_id)
{
  const uint64_t first = GlobalEventIdProvider::get_next_id();
  EXPECT_EQ(first, GlobalEventIdProvider::get_last_id());
  EXPECT_EQ(first + 1, GlobalEventIdProvider::get_next_id());
  EXPECT_EQ(first + 1, GlobalEventIdProvider::get_last_id());
}

TEST(TestGlobalEventIdProvider, concurrent_ids_are_unique)
{
  constexpr size_t kThreads = 4;
  constexpr size_t kIdsPerThread = 10000;
  std::vector<std::vector<uint64_t>> ids(kThreads);
  std::vector<std::thread> threads;
  for (size_t t = 0; t < kThreads; ++t) {
    threads.emplace_back([&ids, t]() {
        ids[t].reserve(kIdsPerThread);
        for (size_t i = 0; i < kIdsPerThread; ++i) {
          ids[t].push_back(GlobalEventIdProvider::get_next_id());
        }
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }

  std::vector<uint64_t> all;
  for (const auto & per_thread : ids) {
    EXPECT_TRUE(std::is_sorted(per_thread.begin(), per_thread.end()));
    all.insert(all.end(), per_thread.begin(), per_thread.end());
  }
  std::sort(all.begin(), all.end());
  EXPECT_EQ(all.end(), std::adjacent_find(all.begin(), all.end()));
}
