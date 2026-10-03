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

#include "rcl_exec/types.hpp"

using rcl_exec::EntityType;

TEST(TestTypes, entity_handle_type_follows_rcl_type)
{
  const auto * sub = reinterpret_cast<const rcl_subscription_t *>(0x10);
  const auto * timer = reinterpret_cast<const rcl_timer_t *>(0x10);
  EXPECT_EQ(EntityType::Subscription, rcl_exec::make_entity_handle(sub).type);
  EXPECT_EQ(EntityType::Timer, rcl_exec::make_entity_handle(timer).type);
  EXPECT_EQ(EntityType::Waitable, rcl_exec::make_waitable_handle(sub).type);
}

TEST(TestTypes, entity_handle_equality_includes_type)
{
  const void * key = reinterpret_cast<const void *>(0x10);
  const auto sub = rcl_exec::make_entity_handle(static_cast<const rcl_subscription_t *>(key));
  const auto waitable = rcl_exec::make_waitable_handle(key);
  EXPECT_EQ(sub, sub);
  EXPECT_NE(sub, waitable);
}
