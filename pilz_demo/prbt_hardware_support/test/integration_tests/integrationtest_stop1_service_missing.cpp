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

#include <gtest/gtest.h>

#include <functional>
#include <map>
#include <string>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

#include <prbt_hardware_support/ros_test_helper.h>

namespace prbt_hardware_support
{
static const std::string RECOVER_SERVICE_NAME{ "recover" };
static const std::string HALT_SERVICE_NAME{ "halt" };
static const std::string HOLD_SERVICE_NAME{ "hold" };
static const std::string UNHOLD_SERVICE_NAME{ "unhold" };

static const std::string OMIT_SERVICE_PARAM_NAME{ "omit_service" };
static const std::string STOP1_EXECUTOR_NODE_NAME{ "/stop1_executor_node" };

static constexpr double WAIT_FOR_NODE_SLEEPTIME_S{ 5.0 };

bool triggerCallbackDummy(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp)
{
  return true;
}

/**
 * @brief Test the stop1 startup with a missing service.
 */
class Stop1ServiceMissingIntegrationTest : public ::testing::Test
{
public:
  /**
   * @brief Gather service-advertising functions for all manipulator services and determine service to omit.
   */
  void SetUp() override;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_{ "~" };

  ros::ServiceServer hold_server_;
  ros::ServiceServer unhold_server_;
  ros::ServiceServer halt_server_;
  ros::ServiceServer recover_server_;

  std::function<void()> omit_advertise_func_;
  //! Map holding all but one service-advertising functions
  std::map<std::string, std::function<void()>> advertise_funcs_;
};

void Stop1ServiceMissingIntegrationTest::SetUp()
{
  ROS_DEBUG("SetUp()");

  advertise_funcs_[RECOVER_SERVICE_NAME] = [this]() {
    this->recover_server_ = this->nh_.advertiseService(RECOVER_SERVICE_NAME, triggerCallbackDummy);
  };
  advertise_funcs_[HALT_SERVICE_NAME] = [this]() {
    this->halt_server_ = this->nh_.advertiseService(HALT_SERVICE_NAME, triggerCallbackDummy);
  };
  advertise_funcs_[UNHOLD_SERVICE_NAME] = [this]() {
    this->unhold_server_ = this->nh_.advertiseService(UNHOLD_SERVICE_NAME, triggerCallbackDummy);
  };
  advertise_funcs_[HOLD_SERVICE_NAME] = [this]() {
    this->hold_server_ = this->nh_.advertiseService(HOLD_SERVICE_NAME, triggerCallbackDummy);
  };

  std::string omit_service;
  ASSERT_TRUE(nh_priv_.getParam(OMIT_SERVICE_PARAM_NAME, omit_service));

  const auto& it = advertise_funcs_.find(omit_service);
  ASSERT_TRUE(it != advertise_funcs_.end()) << "Invalid service name: " << omit_service;

  omit_advertise_func_ = it->second;
  advertise_funcs_.erase(it);
}

/**
 * @brief Test that the startup of the stop1_executor is not complete if a service is missing.
 *
 * @tests{stop1_missing_services,
 *  Test that the startup of the stop1_executor is not complete if a service is missing.
 * }
 *
 * Test Sequence:
 *    1. Advertise all but one manipulator services
 *    2. Advertise missing service from step 1
 *
 * Expected Results:
 *    1. Stop1 executor node does not come up
 *    2. Stop1 executor node does complete startup
 */
TEST_F(Stop1ServiceMissingIntegrationTest, testMissingService)
{
  /**********
   * Step 1 *
   **********/
  ROS_DEBUG("Step 1");

  std::for_each(advertise_funcs_.begin(), advertise_funcs_.end(),
                [](const std::pair<std::string, std::function<void()>>& el) { el.second(); });

  ros::Duration(WAIT_FOR_NODE_SLEEPTIME_S).sleep();
  EXPECT_FALSE(checkForNode(STOP1_EXECUTOR_NODE_NAME));

  /**********
   * Step 2 *
   **********/
  ROS_DEBUG("Step 2");

  omit_advertise_func_();
  waitForNode(STOP1_EXECUTOR_NODE_NAME);
}

}  // namespace prbt_hardware_support

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_stop1_service_missing");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
