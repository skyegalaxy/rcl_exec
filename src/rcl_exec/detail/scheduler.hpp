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

#ifndef RCL_EXEC__DETAIL__SCHEDULER_HPP_
#define RCL_EXEC__DETAIL__SCHEDULER_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

namespace rcl_exec
{
namespace detail
{
struct Worker
{
  std::mutex mutex;
  std::condition_variable condition_variable;
  std::atomic<bool> wakeup = false;

  void prepare_block()
  {
    wakeup = false;
  }

  void block()
  {
    std::unique_lock lk(mutex);
    condition_variable.wait(lk, [this]() -> bool {
        return wakeup;
    });
  }

  void block_for(std::chrono::nanoseconds timeout)
  {
    std::unique_lock lk(mutex);
    condition_variable.wait_for(lk, timeout, [this]() -> bool {
        return wakeup;
    });
  }

  /**
   * This function is for normal unblocking, when there is
   * no risk that the worker might get deleted. This version
   * is more performant, as the notified thread will not directly
   * run into the locked mutex.
   */
  void unblock()
  {
    {
      std::unique_lock lk(mutex);
      wakeup = true;
    }
    condition_variable.notify_one();
  }

  /*
   * This function shall be used if there is a risk that the worker
   * might get deleted after wakeup. This function is supposed to
   * be used in the shutdown case.
   * This function is less performant as the unblock function, as
   * the notofied thread may be tempoarlily blocked again until the
   * mutex is released.
   */
  void unblock_thread_safe()
  {
    std::unique_lock lk(mutex);
    wakeup = true;
    condition_variable.notify_one();
  }
};

/**
 * Queue of blocked (idle) worker threads.
 *
 * All member functions must be called while holding workers_mutex.
 * The mutex is exposed so that the scheduler can lock it together
 * with its own ready_callback_groups_mutex, which is needed to make
 * "check for work and enqueue self" atomic with respect to
 * "add work and wake a worker".
 */
struct WorkerQueue
{
  std::mutex workers_mutex;
  std::deque<Worker *> workers;

  bool release_workers = false;

  /**
   * Removes and returns the most recently blocked worker, or nullptr
   * if no worker is blocked.
   */
  Worker * pop_blocked_worker_thread()
  {
    if(workers.empty()) {
      // no threads available
      return nullptr;
    }
    Worker * worker = workers.front();
    workers.pop_front();
    return worker;
  }

  /**
   * Registers the worker as blocked. Returns false if the queue was
   * released and the worker must not block.
   */
  bool push_blocked_worker_thread(Worker * worker)
  {
    if(release_workers) {
      return false;
    }
    workers.push_front(worker);
    return true;
  }

  /**
   * Removes the given worker from the queue, if it is still in there.
   * Needed for timed blocks, where the worker may wake up on its own
   * without anyone having removed it from the queue.
   */
  void remove_worker_thread(Worker * worker)
  {
    auto it = std::find(workers.begin(), workers.end(), worker);
    if(it != workers.end()) {
      workers.erase(it);
    }
  }

  /**
   * Removes all blocked workers from the queue and marks the queue as
   * released. Returns the removed workers, the caller must unblock them
   * after dropping the lock.
   */
  std::deque<Worker *> release_all_worker_threads()
  {
    std::deque<Worker *> cpy;
    cpy.swap(workers);
    release_workers = true;
    return cpy;
  }
};
}  // namespace detail
}  // namespace rcl_exec

#endif  // RCL_EXEC__DETAIL__SCHEDULER_HPP_
