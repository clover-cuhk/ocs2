/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <condition_variable>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace ocs2 {

/**
 * Thread pool class to execute tasks on multiple threads.
 */
class ThreadPool {
 public:
  /**
   * Constructor
   *
   * @param [in] nThreads: Number of threads to launch in the pool
   * @param [in] priority: The worker thread priority
   */
  explicit ThreadPool(int nThreads = 1, int priority = 0);

  /**
   * Destructor
   */
  ~ThreadPool();

  /**
   * Run a task in another thread
   *
   * @tparam Functor: The task function
   * @param [in] taskFunction: The task function to run in the pool. It takes a thread worker index argument (between 0 and nThreads - 1),
                               which can be used to index designated thread ressources.
   * @return future object with taskFunction retrun value
   */
  template <typename Functor>
  std::future<typename std::result_of<Functor(int)>::type> run(Functor taskFunction);

  /**
   * Helper function to run a task N times parallel on the pool
   *
   * @note This is a blocking operation, returns when all tasks are completed.
   * @warning Calling runParallel(task, nThreads) does not guarantee that each task will be executed with a different workerIndex.
   *
   * @param [in] taskFunction: task function to run in the pool.
   * @param [in] N: number of times to run taskFunction in parallel, if N = 1 the main thread is used.
   */
  void runParallel(std::function<void(int)> taskFunction, int N);

  /**
   * Enable debug log
   *
   * @param [in] debugPrint: re-entrant line printing function
   */
  void enableDebug(std::function<void(const std::string)> debugPrint);

  /** Get the number of threads. */
  size_t numThreads() const { return workerThreads_.size(); }

 private:
  struct TaskBase;

  template <typename Functor>
  struct Task;

  /**
   * Pop next task from ready queue
   *
   * @return pointer to next task to run, nullptr if stop condiditon
   */
  std::unique_ptr<TaskBase> popReady();

  /**
   * Thread worker loop
   *
   * @param [in] workerIndex: worker thread index
   */
  void worker(int workerIndex);

  /**
   * Run a task asynchronously in another thread
   *
   * @param [in] taskPtr: task object
   */
  void runTask(std::unique_ptr<TaskBase> taskPtr);

  bool stop_ = false;  // protected by readyQueueLock_
  std::queue<std::unique_ptr<TaskBase>> taskQueue_;
  std::condition_variable taskQueueCondition_;
  std::mutex taskQueueLock_;

  std::vector<std::thread> workerThreads_;
};

/**
 * Task callback interface class.
 */
struct ThreadPool::TaskBase {
  TaskBase() = default;
  virtual ~TaskBase() = default;
  virtual void operator()(int workerIndex) = 0;
};

/**
 * Task callback for a specific function return type.
 *
 * @tparam Functor: Type of callable task object (functor).
 */
template <typename Functor>
struct ThreadPool::Task final : public ThreadPool::TaskBase {
  explicit Task(Functor taskFunction) : packagedTask(taskFunction) {}
  ~Task() override = default;
  void operator()(int workerIndex) override { packagedTask(workerIndex); }

  using ReturnType = typename std::result_of<Functor(int)>::type;
  std::packaged_task<ReturnType(int)> packagedTask;
};

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::run(Functor taskFunction) {
  auto taskPtr = new Task<Functor>(taskFunction);
  auto future = taskPtr->packagedTask.get_future();
  std::unique_ptr<TaskBase> taskBasePtr(taskPtr);

  if (workerThreads_.empty()) {
    // run on main thread
    taskBasePtr->operator()(0);
  } else {
    runTask(std::move(taskBasePtr));
  }

  return std::move(future);
}

}  // namespace ocs2
