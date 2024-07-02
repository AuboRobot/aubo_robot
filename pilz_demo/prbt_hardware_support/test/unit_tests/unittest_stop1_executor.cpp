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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <pilz_testutils/async_test.h>

#include <prbt_hardware_support/stop1_executor.h>
#include <prbt_hardware_support/service_function_decl.h>

#define EXPECT_RECOVER                                                                                                 \
  EXPECT_CALL(*this, recover_func()).WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_UNHOLD                                                                                                  \
  EXPECT_CALL(*this, unhold_func()).WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(UNHOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HOLD                                                                                                    \
  EXPECT_CALL(*this, hold_func()).WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(true)))

#define EXPECT_HALT                                                                                                    \
  EXPECT_CALL(*this, halt_func()).WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(true)))

namespace prbt_hardware_support_tests
{
using namespace prbt_hardware_support;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

const std::string RECOVER_SRV_CALLED_EVENT{ "recover_srv_called" };
const std::string UNHOLD_SRV_CALLED_EVENT{ "unhold_srv_called" };
const std::string HOLD_SRV_CALLED_EVENT{ "hold_srv_called" };
const std::string HALT_SRV_CALLED_EVENT{ "halt_srv_called" };

/**
 * Gives access to protected methods of run_permitted adapter.
 */
class Stop1ExecutorForTests : public Stop1Executor
{
public:
  Stop1ExecutorForTests(const TServiceCallFunc& hold_func, const TServiceCallFunc& unhold_func,
                        const TServiceCallFunc& recover_func, const TServiceCallFunc& halt_func)
    : Stop1Executor(hold_func, unhold_func, recover_func, halt_func)
  {
  }

  FRIEND_TEST(Stop1ExecutorTest, testExitInStateEnabling);
  FRIEND_TEST(Stop1ExecutorTest, testExitInStateStopping);
  FRIEND_TEST(Stop1ExecutorTest, testExitInStateEnableRequestDuringStop);
  FRIEND_TEST(Stop1ExecutorTest, testExitInStateStopRequestedDuringEnable);
};

class Stop1ExecutorTest : public ::testing::Test, public ::testing::AsyncTest
{
protected:
  Stop1ExecutorForTests* createStop1Executor();

public:
  MOCK_METHOD0(hold_func, bool());
  MOCK_METHOD0(unhold_func, bool());
  MOCK_METHOD0(recover_func, bool());
  MOCK_METHOD0(halt_func, bool());
};

inline Stop1ExecutorForTests* Stop1ExecutorTest::createStop1Executor()
{
  return new Stop1ExecutorForTests(
      std::bind(&Stop1ExecutorTest::hold_func, this), std::bind(&Stop1ExecutorTest::unhold_func, this),
      std::bind(&Stop1ExecutorTest::recover_func, this), std::bind(&Stop1ExecutorTest::halt_func, this));
}

/**
 * @brief Test increases function coverage by ensuring that all Dtor variants
 * are called.
 *
 */
TEST_F(Stop1ExecutorTest, testDestructor)
{
  {
    std::shared_ptr<Stop1Executor> adapter_run_permitted{ new Stop1Executor(
        std::bind(&Stop1ExecutorTest::hold_func, this), std::bind(&Stop1ExecutorTest::unhold_func, this),
        std::bind(&Stop1ExecutorTest::recover_func, this), std::bind(&Stop1ExecutorTest::halt_func, this)) };
  }

  {
    Stop1Executor adapter_run_permitted(
        std::bind(&Stop1ExecutorTest::hold_func, this), std::bind(&Stop1ExecutorTest::unhold_func, this),
        std::bind(&Stop1ExecutorTest::recover_func, this), std::bind(&Stop1ExecutorTest::halt_func, this));
  }
}

/**
 * @tests{release_of_stop1,
 * Tests that driver is recovered and controller no longer holded
 * in case of RUN_PERMITTED switch: false->true.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 */
TEST_F(Stop1ExecutorTest, testEnable)
{
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };
  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });
}

/**
 * @tests{execution_of_stop1,
 * Test enabling stopping and enabling again.
 * }
 *
 * @tests{release_of_stop1,
 * Test enabling stopping and enabling again.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover and unhold services return success
 *  2. Call updateRunPermitted(false)),
 *     let hold and halt services return success
 *  3. Call updateRunPermitted(true)) repeatedly,
 *     let recover and unhold service return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(Stop1ExecutorTest, testEnableStopEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };
  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{ true };
  std::thread spam_enable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(true);
    }
  } };

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @brief Test spaming enable plus subsequent stop
 *
 * @tests{execution_of_stop1,
 * Test spaming enable plus subsequent stop.
 * }
 *
 * @tests{release_of_stop1,
 * Test spaming enable plus subsequent stop.
 * }
 *
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)) repeatedly,
 *     let recover and unhold services return success
 *  2. Call updateRunPermitted(true) once more (this is certainly after unhold and needed for full coverage)
 *  3. Call updateRunPermitted(false)),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. -
 *  3. Hold and halt services are called successively
 */
TEST_F(Stop1ExecutorTest, testSpamEnablePlusStop)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  std::atomic_bool keep_spamming{ true };
  std::thread spam_enable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(true);
    }
  } };

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_enable.join();

  /**********
   * Step 2 *
   **********/

  adapter_run_permitted->updateRunPermitted(true);

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });
}

/**
 * @brief Test spamming RUN_PERMITTED=false plus subsequent enable
 *
 * @tests{execution_of_stop1,
 * Test spamming RUN_PERMITTED=false plus subsequent enable.
 * }
 *
 * @tests{release_of_stop1,
 * Test spamming RUN_PERMITTED=false plus subsequent enable.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover and unhold services return success
 *  2. Call updateRunPermitted(false)) repeatedly,
 *     let hold and halt services return success
 *  3. Call updateRunPermitted(true)) repeatedly,
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold and halt services are called successively
 *  3. Recover and unhold services are called successively
 */
TEST_F(Stop1ExecutorTest, testSpamRunPermittedActivePlusEnable)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_HOLD;
    EXPECT_HALT;
  }

  std::atomic_bool keep_spamming{ true };
  std::thread spam_disable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(false);
    }
  } };

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_disable.join();

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  keep_spamming = true;
  std::thread spam_enable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(true);
    }
  } };

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_enable.join();
}

/**
 *
 * @tests{execution_of_stop1,
 * Test skipping hold when run_permitted changes to false during recover.
 * }
 *
 * @tests{release_of_stop1,
 * Test skipping hold when run_permitted changes to false during recover.
 * }
 *
 * @tests{release_of_stop1_interrupt,
 * Test skipping hold when run_permitted changes to false during recover.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     call updateRunPermitted(false)) during recover service call and return success, let halt service return success
 *  2. Call updateRunPermitted(true)),
 *     let recover and unhold services return success
 *
 * Expected Results:
 *  1. Recover and halt services are called successively
 *  2. Recover and unhold services are called successively
 */
TEST_F(Stop1ExecutorTest, testSkippingHoldPlusEnable)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for recover-invoke action
  auto run_permitted_false_during_recover_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(false);
    return true;
  };

  auto halt_action = [this]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(*this, recover_func()).WillOnce(InvokeWithoutArgs(run_permitted_false_during_recover_action));

    EXPECT_CALL(*this, halt_func()).WillOnce(InvokeWithoutArgs(halt_action));
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{ true };
  std::thread spam_enable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(true);
    }
  } };

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @tests{execution_of_stop1,
 * Test sending run_permitted change to true while hold call is still pending.
 * }
 *
 * @tests{release_of_stop1,
 * Test sending run_permitted change to true while hold call is still pending.
 * }
 *
 * Test Sequence:
 *  1.  Run the run_permitted adapter and call updateRunPermitted(true),
 *      let recover and unhold service return success.
 *  2.  Call updateRunPermitted(false) and updateRunPermitted(true) during hold service call,
 *      let hold, halt, recover and unhold service return success.
 *
 * Expected Results:
 *  1.  Recover and unhold services are called successively.
 *  2.  Hold, halt, recover and unhold services are called successively.
 */
TEST_F(Stop1ExecutorTest, testEnableDuringHoldService)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  auto enable_during_hold_action = [this, &adapter_run_permitted]() {
    adapter_run_permitted->updateRunPermitted(true);
    this->triggerClearEvent(HOLD_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  {
    InSequence dummy;

    EXPECT_CALL(*this, hold_func()).WillOnce(InvokeWithoutArgs(enable_during_hold_action));

    EXPECT_HALT;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT, RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });
}

/**
 * @tests{execution_of_stop1,
 * Test sending run_permitted change to true while halt call is still pending.
 * }
 *
 * @tests{release_of_stop1,
 * Test sending run_permitted change to true while halt call is still pending.
 * }
 *
 * Test Sequence:
 *  1.  Run the run_permitted adapter and call updateRunPermitted(true)),
 *      call updateRunPermitted(false)) during recover service call and return success.
 *      Before returning success on halt service call updateRunPermitted(true)
 *
 * Expected Results:
 *  1.  Recover and halt services are called successively.
 *      Afterwards recover and unhold are called.
 */
TEST_F(Stop1ExecutorTest, testEnableDuringHaltService)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for recover-invoke action
  auto run_permitted_false_during_recover_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(false);
    return true;
  };

  auto enable_during_halt_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(true);
    return true;
  };

  /**********
   * Step 1 *
   **********/

  const std::string recover_srv_called_event2{ "recover_srv_called2" };
  const std::string unhold_srv_called_event2{ "unhold_srv_called2" };
  {
    InSequence dummy;

    EXPECT_CALL(*this, recover_func()).WillOnce(InvokeWithoutArgs(run_permitted_false_during_recover_action));

    EXPECT_CALL(*this, halt_func()).WillOnce(InvokeWithoutArgs(enable_during_halt_action));

    EXPECT_CALL(*this, recover_func()).WillOnce(InvokeWithoutArgs([this, recover_srv_called_event2]() {
      this->triggerClearEvent(recover_srv_called_event2);
      return true;
    }));

    EXPECT_CALL(*this, unhold_func()).WillOnce(InvokeWithoutArgs([this, unhold_srv_called_event2]() {
      this->triggerClearEvent(unhold_srv_called_event2);
      return true;
    }));
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT, recover_srv_called_event2, unhold_srv_called_event2 });
}

/**
 * @tests{execution_of_stop1,
 * Test sending run_permitted change to true and back to false while halt call
 * is still pending.
 * }
 *
 * @tests{release_of_stop1,
 * Test sending run_permitted change to true and back to false while halt call
 * is still pending.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)).
 *     Call updateRunPermitted(false)) during recover service call and return success. (This is not essential for the
 * test) Before returning success on halt service call updateRunPermitted(true) and update(false).
 *
 * Expected Results:
 *  1. Recover and halt services are called successively. Afterwards recover and unhold are called once.
 */
TEST_F(Stop1ExecutorTest, testEnableDisableDuringHaltService)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for recover-invoke action
  auto run_permitted_false_during_recover_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(false);
    return true;
  };

  auto enable_during_halt_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(HALT_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(true);
    adapter_run_permitted->updateRunPermitted(false);  // Important flip!
    return true;
  };

  /**********
   * Step 1 *
   **********/

  {
    InSequence dummy;

    EXPECT_CALL(*this, recover_func()).Times(1).WillOnce(InvokeWithoutArgs(run_permitted_false_during_recover_action));

    EXPECT_CALL(*this, halt_func()).Times(1).WillOnce(InvokeWithoutArgs(enable_during_halt_action));
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });
}

/**
 * @tests{execution_of_stop1,
 * Test enabling with failing recover service and retry (stop plus enable).
 * }
 * @tests{release_of_stop1,
 * Test enabling with failing recover service and retry (stop plus enable).
 * }
 * @tests{release_of_stop1_interrupt,
 * Test enabling with failing recover service and retry (stop plus enable).
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover service fail repeatedly, let unhold service return success
 *  2. Call updateRunPermitted(false)),
 *     let hold and halt service return success
 *  3. Call updateRunPermitted(true)) repeatedly,
 *     recover and unhold service return success
 *
 * Expected Results:
 *  1. Recover service is called at least once
 *  2. Halt service is called
 *  2. Recover and unhold services are called successively
 */
TEST_F(Stop1ExecutorTest, testRecoverFailPlusRetry)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(*this, recover_func())
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(RECOVER_SRV_CALLED_EVENT), Return(false)))
      .WillRepeatedly(Return(false));

  // unhold is optional here
  EXPECT_CALL(*this, unhold_func()).WillRepeatedly(Return(true));

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    // hold and is_executing and is optional here
    EXPECT_CALL(*this, hold_func()).WillRepeatedly(Return(true));

    EXPECT_HALT;
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HALT_SRV_CALLED_EVENT });

  /**********
   * Step 3 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::atomic_bool keep_spamming{ true };
  std::thread spam_enable{ [&adapter_run_permitted, &keep_spamming]() {
    while (keep_spamming)
    {
      adapter_run_permitted->updateRunPermitted(true);
    }
  } };

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  keep_spamming = false;
  spam_enable.join();
}

/**
 * @tests{unhold_service_fails,
 * Test if a stop is possible after unhold failed.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover service return success and unhold service fail repeatedly
 *  2. Call updateRunPermitted(false)),
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively, the latter one at least once
 *  2. Hold and halt services are called successively.
 */
TEST_F(Stop1ExecutorTest, testUnholdFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;

    EXPECT_CALL(*this, unhold_func())
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(UNHOLD_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };
  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });
}

/**
 * @tests{hold_service_fail,
 * Test if stop is continued properly despite failing hold service.
 * }
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     let recover and unhold services return success
 *  2. Call updateRunPermitted(false)),
 *     let hold service fail, let is_executing service return "not executing",
 *     let halt service fail (for full coverage)
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold, is_executing and halt services are called successively
 */
TEST_F(Stop1ExecutorTest, testHoldFail)
{
  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;
    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(*this, hold_func())
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HOLD_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));

    EXPECT_CALL(*this, halt_func())
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(HALT_SRV_CALLED_EVENT), Return(false)))
        .WillRepeatedly(Return(false));
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });
}

/**
 * @brief Test hold immediately when run_permitted changes to false during unhold.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     call updateRunPermitted(false)) during unhold service call and return success,
 *     let hold and halt services return success
 *
 * Expected Results:
 *  1. Recover, unhold, hold and halt services are called successively
 */
TEST_F(Stop1ExecutorTest, testHoldImmediatelyAfterUnhold)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for unhold-invoke action
  auto unhold_action = [this, &adapter_run_permitted]() {
    this->triggerClearEvent(UNHOLD_SRV_CALLED_EVENT);
    adapter_run_permitted->updateRunPermitted(false);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;

    EXPECT_CALL(*this, unhold_func()).WillOnce(InvokeWithoutArgs(unhold_action));

    EXPECT_HOLD;
    EXPECT_HALT;
  }

  adapter_run_permitted->updateRunPermitted(true);

  barricade({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT, HOLD_SRV_CALLED_EVENT, HALT_SRV_CALLED_EVENT });
}

/**
 * @brief Test stopping the state machine in state Enabling.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true),
 *     call stopStateMachine() during recover service call and return success,
 *     let unhold service return success
 *
 * Expected Results:
 *  1. Recover service is called
 */
TEST_F(Stop1ExecutorTest, testExitInStateEnabling)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for recover-invoke action
  auto recover_action = [this, &adapter_run_permitted]() {
    adapter_run_permitted->stopStateMachine();
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    return true;
  };

  {
    InSequence dummy;

    EXPECT_CALL(*this, recover_func()).WillOnce(InvokeWithoutArgs(recover_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(*this, unhold_func()).WillRepeatedly(Return(true));
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT });
}

/**
 * @brief Test stopping the state machine in state Stopping.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true),
 *     let recover and unhold services return success.
 *  2. Call updateRunPermitted(false) and call stopStateMachine() during hold service call,
 *     let hold and halt services return success.
 *
 * Expected Results:
 *  1. Recover and unhold services are called
 *  2. Hold service is called
 */
TEST_F(Stop1ExecutorTest, testExitInStateStopping)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/
  // define function for hold-invoke action
  auto hold_action = [this, &adapter_run_permitted]() {
    adapter_run_permitted->stopStateMachine();
    this->triggerClearEvent(HOLD_SRV_CALLED_EVENT);
    return true;
  };

  {
    InSequence dummy;

    EXPECT_CALL(*this, hold_func()).WillOnce(InvokeWithoutArgs(hold_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(*this, halt_func()).WillRepeatedly(Return(true));
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER({ HOLD_SRV_CALLED_EVENT });
}

/**
 * @brief Test stopping the state machine in state StopRequestedDuringEnable.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true)),
 *     call updateRunPermitted(false)) and stopStateMachine() during recover service call,
 *     let recover and halt services return success.
 *
 * Expected Results:
 *  1. Recover is called.
 */
TEST_F(Stop1ExecutorTest, testExitInStateStopRequestedDuringEnable)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  // define function for recover-invoke action
  auto recover_action = [this, &adapter_run_permitted]() {
    adapter_run_permitted->updateRunPermitted(false);
    adapter_run_permitted->stopStateMachine();
    this->triggerClearEvent(RECOVER_SRV_CALLED_EVENT);
    return true;
  };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_CALL(*this, recover_func()).WillOnce(InvokeWithoutArgs(recover_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(*this, halt_func()).WillRepeatedly(Return(true));
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER(RECOVER_SRV_CALLED_EVENT);
}

/**
 * @brief Test stopping the state machine in state EnableRequestDuringStop.
 *
 * @note This test exists mainly for full function coverage.
 *
 * Test Sequence:
 *  1. Run the run_permitted adapter and call updateRunPermitted(true),
 *     let recover and unhold services return success
 *  2. Call updateRunPermitted(false),
 *     call updateRunPermitted(true) and stopStateMachine() during hold service call
 *     and return success, let halt, recover and unhold services also return success
 *
 * Expected Results:
 *  1. Recover and unhold services are called successively
 *  2. Hold service is called
 */
TEST_F(Stop1ExecutorTest, testExitInStateEnableRequestDuringStop)
{
  std::unique_ptr<Stop1ExecutorForTests> adapter_run_permitted{ createStop1Executor() };

  /**********
   * Step 1 *
   **********/
  {
    InSequence dummy;

    EXPECT_RECOVER;
    EXPECT_UNHOLD;
  }

  adapter_run_permitted->updateRunPermitted(true);

  BARRIER({ RECOVER_SRV_CALLED_EVENT, UNHOLD_SRV_CALLED_EVENT });

  /**********
   * Step 2 *
   **********/

  auto enable_during_hold_action = [this, &adapter_run_permitted]() {
    adapter_run_permitted->updateRunPermitted(true);
    adapter_run_permitted->stopStateMachine();
    this->triggerClearEvent(HOLD_SRV_CALLED_EVENT);
    return true;
  };

  {
    InSequence dummy;

    EXPECT_CALL(*this, hold_func()).WillOnce(InvokeWithoutArgs(enable_during_hold_action));

    // do not exclude other service calls as stopping the state machine does not prevent further actions
    EXPECT_CALL(*this, halt_func()).WillRepeatedly(Return(true));
    EXPECT_CALL(*this, recover_func()).WillRepeatedly(Return(true));
    EXPECT_CALL(*this, unhold_func()).WillRepeatedly(Return(true));
  }

  adapter_run_permitted->updateRunPermitted(false);

  BARRIER(HOLD_SRV_CALLED_EVENT);
}

}  // namespace prbt_hardware_support_tests

int main(int argc, char** argv)
{
  // for (limited) ros::Time functionality, no ROS communication
  ros::Time::init();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
