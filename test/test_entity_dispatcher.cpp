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

#include <memory>
#include <type_traits>
#include <utility>

#include "rcl_exec/entity_dispatcher.hpp"

namespace
{

struct CountingPayload : rcl_exec::TakenPayload
{
  explicit CountingPayload(int & destroyed)
  : destroyed_(destroyed) {}
  ~CountingPayload() override {++destroyed_;}
  int & destroyed_;
};

}  // namespace

TEST(TestEntityDispatcher, taken_data_is_move_only_and_destroys_payload)
{
  static_assert(!std::is_copy_constructible_v<rcl_exec::TakenData>);
  static_assert(std::is_nothrow_move_constructible_v<rcl_exec::TakenData>);

  int destroyed = 0;
  {
    rcl_exec::TakenData data = std::make_unique<CountingPayload>(destroyed);
    rcl_exec::TakenData moved = std::move(data);
    EXPECT_EQ(0, destroyed);
  }
  EXPECT_EQ(1, destroyed);
}
