/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <prbt_hardware_support/stop1_executor.h>

namespace prbt_hardware_support
{
Stop1Executor::Stop1Executor(const TServiceCallFunc& hold_func, const TServiceCallFunc& unhold_func,
                             const TServiceCallFunc& recover_func, const TServiceCallFunc& halt_func)
{
  state_machine_ = std::unique_ptr<RunPermittedStateMachine>(
      new RunPermittedStateMachine(recover_func, halt_func, hold_func, unhold_func));

  state_machine_->start();

  worker_thread_ = std::thread(&Stop1Executor::workerThreadFun, this);
}

Stop1Executor::~Stop1Executor()
{
  if (worker_thread_.joinable())
  {
    {
      std::lock_guard<std::mutex> lock(sm_mutex_);
      terminate_ = true;
    }
    worker_cv_.notify_one();
    worker_thread_.join();
  }

  stopStateMachine();
}

void Stop1Executor::updateRunPermitted(const bool run_permitted)
{
  ROS_DEBUG_STREAM("updateRunPermitted(" << std::boolalpha << run_permitted << std::noboolalpha << ")");
  {
    std::lock_guard<std::mutex> lock(sm_mutex_);
    state_machine_->process_event(typename RunPermittedStateMachine::run_permitted_updated(run_permitted));
  }
  worker_cv_.notify_one();
}

bool Stop1Executor::updateRunPermittedCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  updateRunPermitted(req.data);
  res.success = true;
  return true;
}

void Stop1Executor::workerThreadFun()
{
  std::unique_lock<std::mutex> sm_lock(sm_mutex_);
  while (!terminate_)
  {
    worker_cv_.wait(sm_lock, [this]() { return (!this->state_machine_->task_queue_.empty() || this->terminate_); });
    if (terminate_)
    {
      break;
    }

    AsyncRunPermittedTask task = state_machine_->task_queue_.front();
    state_machine_->task_queue_.pop();

    sm_lock.unlock();  // | This part is executed async from
    task.execute();    // | the state machine since new run_permitted updates need to be handled
    // | during service calls.
    sm_lock.lock();  // |

    task.signalCompletion();  // Could add Task to Queue and does process_event on the state machine. Needs lock.
  }
}

}  // namespace prbt_hardware_support
