/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#ifndef STOP1_EXECUTOR_H
#define STOP1_EXECUTOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <ros/service_client.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <prbt_hardware_support/run_permitted_state_machine.h>
#include <prbt_hardware_support/service_function_decl.h>

namespace prbt_hardware_support
{
/**
 * @brief Performs service calls for Stop1 and the respective reversal,
 * that is enabling the manipulator. Incoming
 * updates of the RUN_PERMITTED state are handled asynchronously.
 *
 * In order to handle the asynchrony of events, a state machine is used to
 * represent the current state. The state machine
 * manages a task queue for the currently required service call.
 *
 * General behaviour:
 *  - RUN_PERMITTED == false:   perfom Stop1 (hold controller + halt drives)
 *  - RUN_PERMITTED == true:    recover drives + unhold controller
 *
 * @note Unhold the controller is skipped if RUN_PERMITTED changes during recover.
 * This avoids the superfluous execution of a hold trajectory, which would
 * result in an overlong stopping time.
 *
 * @note If a service call fails, the execution is always continued in order
 * to make a Stop1 or a recover-retry possible.
 *
 * @remark this class is templated for easier mocking. However for usability
 * it can be used by AdapterSto
 */
class Stop1Executor
{
public:
  /**
   * @brief Create required service clients and state machine;
   * start worker-thread and state machine.
   *
   */
  Stop1Executor(const TServiceCallFunc& hold_func, const TServiceCallFunc& unhold_func,
                const TServiceCallFunc& recover_func, const TServiceCallFunc& halt_func);

  /**
   * @brief Stop state machine and terminate worker-thread.
   */
  virtual ~Stop1Executor();

  /**
   * @brief This is called everytime an updated run_permitted value is obtained.
   *
   * Process run_permitted_updated event and notify worker-thread in case it is waiting
   * for new required tasks.
   *
   * @note
   * Access to the state machine is protected for thread-safety.
   *
   * @param run_permitted The updated run_permitted value.
   */
  void updateRunPermitted(const bool run_permitted);
  bool updateRunPermittedCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

protected:
  /**
   * @brief Stop the state machine.
   *
   * @note The access modifier protected allows this method to be used in tests.
   */
  void stopStateMachine();

private:
  /**
   * @brief This is executed in the worker-thread and allows asynchronous
   * handling of run_permitted updates.
   *
   * Wait for notification if the task queue of the state machine is empty.
   * Once a task is present, execute it and signal its completion.
   *
   * @note
   * Access to the state machine is protected for thread-safety.
   * It is assumed that task execution does not access
   * the state machine, whereas the completion signalling does.
   */
  void workerThreadFun();

private:
  //! State machine
  std::unique_ptr<RunPermittedStateMachine> state_machine_;

  //! Flag indicating if the worker-thread should terminate
  std::atomic_bool terminate_{ false };

  //! Mutex for protecting access to the state machine, needs to be owned when
  //! triggering an event of the state machine
  std::mutex sm_mutex_;

  //! Condition variable for notifying a waiting worker-thread
  std::condition_variable worker_cv_;

  //! Worker-thread
  std::thread worker_thread_;
};

inline void Stop1Executor::stopStateMachine()
{
  ROS_DEBUG("Stop state machine");
  state_machine_->stop();
}

}  // namespace prbt_hardware_support

#endif  // STOP1_EXECUTOR_H
