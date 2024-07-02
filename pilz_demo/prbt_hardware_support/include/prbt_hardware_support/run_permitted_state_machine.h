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

#ifndef PRBT_HARDWARE_SUPPORT_RUN_PERMITTED_STATE_MACHINE_H
#define PRBT_HARDWARE_SUPPORT_RUN_PERMITTED_STATE_MACHINE_H

#include <queue>
#include <string>

#include <ros/ros.h>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/core/demangle.hpp>
#include <ros/console.h>

#include <prbt_hardware_support/service_function_decl.h>
#include <prbt_hardware_support/utils.h>

namespace prbt_hardware_support
{
#define COLOR_GREEN "\033[32m"
#define COLOR_GREEN_BOLD "\033[1;32m"

#define STATE_ENTER_OUTPUT                                                                                             \
  ROS_DEBUG_STREAM_NAMED("RunPermittedStateMachine",                                                                   \
                         "Event: " << className(boost::core::demangle(typeid(ev).name()))                              \
                                   << " - Entering: " << COLOR_GREEN_BOLD                                              \
                                   << className(boost::core::demangle(typeid(*this).name())) << COLOR_GREEN);
#define STATE_EXIT_OUTPUT                                                                                              \
  ROS_DEBUG_STREAM_NAMED("RunPermittedStateMachine",                                                                   \
                         "Event: " << className(boost::core::demangle(typeid(ev).name()))                              \
                                   << " - Leaving: " << className(boost::core::demangle(typeid(*this).name())));
#define ACTION_OUTPUT                                                                                                  \
  ROS_DEBUG_STREAM_NAMED("RunPermittedStateMachine",                                                                   \
                         "Event: " << className(boost::core::demangle(typeid(ev).name()))                              \
                                   << " - Action: " << className(boost::core::demangle(typeid(*this).name())));

/**
 * @brief An AsyncRunPermittedTask is represented by a task execution and a completion signalling.
 *
 * The separation of task execution and the completion signalling allows the task execution to be done asynchronously.
 * Both functions have the signature @code void() @endcode.
 */
class AsyncRunPermittedTask
{
public:
  AsyncRunPermittedTask(const TServiceCallFunc& operation, const std::function<void()>& finished_handler)
    : operation_(operation), finished_handler_(finished_handler)
  {
  }

  /**
   * @brief Execute the task.
   */
  void execute()
  {
    if (operation_)
    {
      operation_();
    }
  }

  /**
   * @brief Signal completion of the task execution.
   */
  void signalCompletion()
  {
    finished_handler_();
  }

private:
  TServiceCallFunc operation_;
  std::function<void()> finished_handler_;
};

//! Define the task queue type
using RunPermittedTaskQueue = std::queue<AsyncRunPermittedTask>;

namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace msm::front;

/**
 * @brief Front-end state machine.
 *
 * Defines states, events, guards, actions and transitions. Pushes AsyncRunPermittedTask's on a task queue.
 *
 * @note
 * This code is not thread-safe.
 */
class RunPermittedStateMachine_ : public msm::front::state_machine_def<RunPermittedStateMachine_>  // CRTP
{
public:
  /**
   * @brief Construct the front-end state machine. Store the required task execution functions.
   *
   * @param recover_operation The execution function of the recover-task.
   * @param halt_operation The execution function of the halt-task.
   * @param hold_operation The execution function of the hold-task.
   * @param unhold_operation The execution function of the unhold-task.
   */
  RunPermittedStateMachine_(const TServiceCallFunc& recover_operation, const TServiceCallFunc& halt_operation,
                            const TServiceCallFunc& hold_operation, const TServiceCallFunc& unhold_operation)
    : recover_op_(recover_operation), halt_op_(halt_operation), hold_op_(hold_operation), unhold_op_(unhold_operation)
  {
  }

  ////////////
  // States //
  ////////////

  struct RobotInactive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };
  struct RobotActive : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };

  struct Enabling : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };

  struct Stopping : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };

  struct StopRequestedDuringEnable : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };

  struct EnableRequestedDuringStop : public msm::front::state<>
  {
    template <class Event, class FSM>
    void on_entry(Event const& ev, FSM&)
    {
      STATE_ENTER_OUTPUT
    }
    template <class Event, class FSM>
    void on_exit(Event const& ev, FSM&)
    {
      STATE_EXIT_OUTPUT
    }
  };

  //! Initial state
  typedef RobotInactive initial_state;

  ////////////
  // Events //
  ////////////

  /**
   * @brief Holds the updated run_permitted value.
   */
  struct run_permitted_updated
  {
    run_permitted_updated(const bool run_permitted) : run_permitted_(run_permitted)
    {
    }

    bool run_permitted_;
  };

  struct recover_done
  {
  };

  struct halt_done
  {
  };

  struct hold_done
  {
  };

  struct unhold_done
  {
  };

  ////////////
  // Guards //
  ////////////

  struct run_permitted_true
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM&, SourceState&, TargetState&)
    {
      return evt.run_permitted_;
    }
  };

  struct run_permitted_false
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM&, SourceState&, TargetState&)
    {
      return !evt.run_permitted_;
    }
  };

  /////////////
  // Actions //
  /////////////

  /**
   * @brief Pushes the recover-task on the task queue.
   *
   * The recover task consists of a recover operation and processing the recover_done event.
   */
  struct recover_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& ev, FSM& fsm, SourceState&, TargetState&)
    {
      ACTION_OUTPUT

      fsm.task_queue_.push(AsyncRunPermittedTask(fsm.recover_op_, [&fsm]() { fsm.process_event(recover_done()); }));
    }
  };

  /**
   * @brief Pushes the halt-task on the task queue.
   *
   * The halt task consists of a halt operation and processing the halt_done event.
   */
  struct halt_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& ev, FSM& fsm, SourceState&, TargetState&)
    {
      ACTION_OUTPUT

      fsm.task_queue_.push(AsyncRunPermittedTask(fsm.halt_op_, [&fsm]() { fsm.process_event(halt_done()); }));
    }
  };

  /**
   * @brief Pushes the hold-task on the task queue.
   *
   * The hold task consists of a hold operation and processing the hold_done event.
   */
  struct hold_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& ev, FSM& fsm, SourceState&, TargetState&)
    {
      ACTION_OUTPUT

      fsm.task_queue_.push(AsyncRunPermittedTask(fsm.hold_op_, [&fsm]() { fsm.process_event(hold_done()); }));
    }
  };

  /**
   * @brief Pushes the unhold-task on the task queue.
   *
   * The unhold task consists of a unhold operation and processing the unhold_done event.
   */
  struct unhold_start
  {
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& ev, FSM& fsm, SourceState&, TargetState&)
    {
      ACTION_OUTPUT

      fsm.task_queue_.push(AsyncRunPermittedTask(fsm.unhold_op_, [&fsm]() { fsm.process_event(unhold_done()); }));
    }
  };

  /////////////////
  // Transitions //
  /////////////////

  struct transition_table
    : mpl::vector<
          //  Start                       Event          Target                      Action         Guard
          // +---------------------------+--------------+---------------------------+--------------+----------+
          Row<RobotInactive, run_permitted_updated, Enabling, recover_start, run_permitted_true>,
          Row<RobotInactive, run_permitted_updated, none, none, run_permitted_false>,
          Row<Enabling, run_permitted_updated, none, none, run_permitted_true>,
          Row<Enabling, run_permitted_updated, StopRequestedDuringEnable, none, run_permitted_false>,
          Row<Enabling, recover_done, none, unhold_start, none>, Row<Enabling, unhold_done, RobotActive, none, none>,
          Row<StopRequestedDuringEnable, run_permitted_updated, none, none, none>,
          Row<StopRequestedDuringEnable, recover_done, Stopping, halt_start, none>,
          Row<StopRequestedDuringEnable, unhold_done, Stopping, hold_start, none>,
          Row<RobotActive, run_permitted_updated, none, none, run_permitted_true>,
          Row<RobotActive, run_permitted_updated, Stopping, hold_start, run_permitted_false>,
          Row<Stopping, run_permitted_updated, EnableRequestedDuringStop, none, run_permitted_true>,
          Row<Stopping, hold_done, none, halt_start, none>, Row<Stopping, halt_done, RobotInactive, none, none>,
          Row<EnableRequestedDuringStop, hold_done, none, halt_start, none>,
          Row<EnableRequestedDuringStop, halt_done, Enabling, recover_start, none>,
          Row<EnableRequestedDuringStop, run_permitted_updated, Stopping, none, run_permitted_false>,
          Row<EnableRequestedDuringStop, run_permitted_updated, none, none, run_permitted_true>
          // +---------------------------+--------------+---------------------------+--------------+----------+
          >
  {
  };

  //! The task queue
  RunPermittedTaskQueue task_queue_;

  //! The recover operation
  TServiceCallFunc recover_op_;

  //! The halt operation
  TServiceCallFunc halt_op_;

  //! The hold operation
  TServiceCallFunc hold_op_;

  //! The unhold operation
  TServiceCallFunc unhold_op_;
};

//! The top-level (back-end) state machine
typedef msm::back::state_machine<RunPermittedStateMachine_> RunPermittedStateMachine;

}  // namespace prbt_hardware_support

#endif  // PRBT_HARDWARE_SUPPORT_RUN_PERMITTED_STATE_MACHINE_H
