cmake_minimum_required(VERSION 3.0.2)
project(prbt_hardware_support)

find_package(catkin REQUIRED COMPONENTS
#  canopen_chain_node
  message_filters
  message_generation
#  pilz_utils
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  dynamic_reconfigure
  pilz_msgs
)

add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)
add_definitions(-std=c++11)
add_definitions(-Wconversion) # At least this line needs to be below find_package to keep
                              #  the flag away from the gmock build
add_definitions(-Wpedantic)

# message generation
add_message_files(
  FILES
  BrakeTestErrorCodes.msg
  ModbusMsgInStamped.msg
  ModbusRegisterBlock.msg
)

# action generation
add_service_files(
  FILES
  BrakeTest.srv
  SendBrakeTestResult.srv
  WriteModbusRegister.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/FakeSpeedOverride.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime pilz_msgs roscpp std_msgs std_srvs sensor_msgs
)

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()


###########
## Build ##
###########

include_directories(
  include
  test/include
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS}) #Must be declared SYSTEM to avoid Warnings from ros system includes
#include_directories(${pilz_utils_INCLUDE_DIRS})


# MODBUS_ADAPTER_RUN_PERMITTED_NODE
#add_executable(modbus_adapter_run_permitted_node
#  src/modbus_adapter_run_permitted_node.cpp
#  src/modbus_adapter_run_permitted.cpp
#  src/modbus_msg_run_permitted_wrapper.cpp
#)
#add_dependencies(modbus_adapter_run_permitted_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(modbus_adapter_run_permitted_node ${catkin_LIBRARIES})

# STOP1_EXECUTOR_NODE
#add_executable(stop1_executor_node
#  src/stop1_executor.cpp
#  src/stop1_executor_node.cpp
#)
#add_dependencies(stop1_executor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(stop1_executor_node ${catkin_LIBRARIES})

# OPERATION_MODE_SETUP_EXECUTOR_NODE
#add_executable(operation_mode_setup_executor_node
#  src/operation_mode_setup_executor.cpp
#  src/operation_mode_setup_executor_node.cpp
#)
#add_dependencies(operation_mode_setup_executor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(operation_mode_setup_executor_node ${catkin_LIBRARIES})

# BRAKE_TEST_NODE
#add_executable(modbus_adapter_brake_test_node
#  src/modbus_adapter_brake_test_node.cpp
#  src/modbus_adapter_brake_test.cpp
#)
#add_dependencies(modbus_adapter_brake_test_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(modbus_adapter_brake_test_node ${catkin_LIBRARIES})

add_executable(modbus_adapter_operation_mode_node
  src/modbus_adapter_operation_mode_node.cpp
  src/modbus_adapter_operation_mode.cpp
  src/adapter_operation_mode.cpp
)

add_dependencies(modbus_adapter_operation_mode_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(modbus_adapter_operation_mode_node ${catkin_LIBRARIES})

#add_executable(brake_test_executor_node
#  src/brake_test_executor_node.cpp
#  src/brake_test_executor.cpp
#)
#add_dependencies(brake_test_executor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(brake_test_executor_node ${catkin_LIBRARIES})

#add_executable(canopen_braketest_adapter_node
#  src/canopen_braketest_adapter_node.cpp
#  src/canopen_braketest_adapter.cpp
#  src/canopen_braketest_adapter_exception.cpp
#)
#add_dependencies(canopen_braketest_adapter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(canopen_braketest_adapter_node ${catkin_LIBRARIES})

add_executable(fake_speed_override_node
  src/fake_speed_override_node.cpp
)
add_dependencies(fake_speed_override_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(fake_speed_override_node ${catkin_LIBRARIES})

# +++++++++++++++++++++++++++++++++
# + Build modbus client node +
# +++++++++++++++++++++++++++++++++
#add_executable(
#  pilz_modbus_client_node
#  src/pilz_modbus_client_node.cpp
#  src/pilz_modbus_client.cpp
#  src/libmodbus_client.cpp
#  src/modbus_check_ip_connection.cpp
#  src/modbus_msg_in_builder.cpp
#)
#add_dependencies(pilz_modbus_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(pilz_modbus_client_node ${catkin_LIBRARIES} modbus)


#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

install(TARGETS
#  brake_test_executor_node
#  canopen_braketest_adapter_node
  fake_speed_override_node
#  modbus_adapter_brake_test_node
  modbus_adapter_operation_mode_node
#  modbus_adapter_run_permitted_node
#  operation_mode_setup_executor_node
#  pilz_modbus_client_node
#  stop1_executor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

#  find_package(pilz_testutils REQUIRED)

#  include_directories(${pilz_testutils_INCLUDE_DIRS})

  #catkin_lint: ignore_once missing_directory <-- needed for successful catkin_linting
  include_directories(SYSTEM ${gmock_SOURCE_DIR}/include/) #Declare system to supress strict conversion errors in gmock

  add_rostest_gmock(unittest_update_filter
                    test/unit_tests/unittest_update_filter.test
                    test/unit_tests/unittest_update_filter.cpp
  )
  target_link_libraries(unittest_update_filter ${catkin_LIBRARIES})
  add_dependencies(unittest_update_filter ${${PROJECT_NAME}_EXPORTED_TARGETS})

  catkin_add_gtest(unittest_libmodbus_client
      test/unit_tests/unittest_libmodbus_client.cpp
      test/unit_tests/pilz_modbus_server_mock.cpp
      src/libmodbus_client.cpp
      src/modbus_check_ip_connection.cpp
  )
  target_link_libraries(unittest_libmodbus_client
    ${catkin_LIBRARIES}
    modbus
  )
  
  catkin_add_gtest(unittest_modbus_check_ip_connection
      test/unit_tests/unittest_modbus_check_ip_connection.cpp
      test/unit_tests/pilz_modbus_server_mock.cpp
      src/modbus_check_ip_connection.cpp
      src/libmodbus_client.cpp
  )
  target_link_libraries(unittest_modbus_check_ip_connection
    ${catkin_LIBRARIES}
    modbus
  )

  catkin_add_gmock(unittest_modbus_api_spec
    test/unit_tests/unittest_modbus_api_spec.cpp
  )
  target_link_libraries(unittest_modbus_api_spec ${catkin_LIBRARIES})

  catkin_add_gtest(unittest_pilz_modbus_exception
                   test/unit_tests/unittest_pilz_modbus_exception.cpp)

  #--- ModbusMsgWrapper unit test ---
  catkin_add_gtest(unittest_modbus_msg_wrapper
                   test/unit_tests/unittest_modbus_msg_wrapper.cpp
                   src/modbus_msg_in_builder.cpp
                   )
  target_link_libraries(unittest_update_filter ${catkin_LIBRARIES})
  add_dependencies(unittest_modbus_msg_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------

#  catkin_add_gtest(unittest_pilz_modbus_client_exception
#    test/unit_tests/unittest_pilz_modbus_client_exception.cpp)

  #--- PilzModbusClient unit test ---
#  add_rostest_gmock(
#       unittest_pilz_modbus_client
#      test/unit_tests/unittest_pilz_modbus_client.test
#      test/unit_tests/unittest_pilz_modbus_client.cpp
#      src/pilz_modbus_client.cpp
#      src/modbus_msg_in_builder.cpp
#  )
#  target_link_libraries(unittest_pilz_modbus_client
#    ${catkin_LIBRARIES}
#  )
#  add_dependencies(unittest_pilz_modbus_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

  # --- ModbusAdapterBrakeTest unit test ---
#  catkin_add_gmock(unittest_modbus_adapter_brake_test
#    test/unit_tests/unittest_modbus_adapter_brake_test.cpp
#    src/modbus_adapter_brake_test.cpp
#    src/modbus_msg_in_builder.cpp
#  )
#  target_link_libraries(unittest_modbus_adapter_brake_test
#    ${catkin_LIBRARIES}
#  )
#  add_dependencies(unittest_modbus_adapter_brake_test ${catkin_EXPORTED_TARGETS})
  #----------------------------------

  # --- ModbusAdapterOperationMode unit test ---
  add_rostest_gmock(unittest_modbus_adapter_operation_mode
    test/unit_tests/unittest_modbus_adapter_operation_mode.test
    test/unit_tests/unittest_modbus_adapter_operation_mode.cpp
    src/modbus_adapter_operation_mode.cpp
    src/adapter_operation_mode.cpp
    src/modbus_msg_in_builder.cpp
  )
  target_link_libraries(unittest_modbus_adapter_operation_mode
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_modbus_adapter_operation_mode ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------

  # --- BrakeTestUtils unit test ---
  add_rostest_gtest(unittest_brake_test_utils
    test/unit_tests/unittest_brake_test_utils.test
    test/unit_tests/unittest_brake_test_utils.cpp
  )
  target_link_libraries(unittest_brake_test_utils
    ${catkin_LIBRARIES} ${pilz_testutils_LIBRARIES}
  )
  #----------------------------------

  # --- CanOpenBraketestAdapter unit test ---
#  add_rostest_gmock(unittest_canopen_braketest_adapter
#    test/unit_tests/unittest_canopen_braketest_adapter.test
#    test/unit_tests/unittest_canopen_braketest_adapter.cpp
#    test/unit_tests/canopen_chain_node_mock.cpp
#    src/canopen_braketest_adapter.cpp
#    src/canopen_braketest_adapter_exception.cpp
#  )
#  target_link_libraries(unittest_canopen_braketest_adapter ${catkin_LIBRARIES})
#  add_dependencies(unittest_canopen_braketest_adapter ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------

  # --- BraketestExecutor unit test ---
#  catkin_add_gmock(unittest_brake_test_executor
#    test/unit_tests/unittest_brake_test_executor.cpp
#    src/brake_test_executor.cpp
#  )
#  target_link_libraries(unittest_brake_test_executor ${catkin_LIBRARIES})
#  add_dependencies(unittest_brake_test_executor ${${PROJECT_NAME}_EXPORTED_TARGETS})
#  add_dependencies(unittest_brake_test_executor ${catkin_EXPORTED_TARGETS})
  #----------------------------------

  # --- Stop1Executor unit test ---
#  catkin_add_gmock(unittest_stop1_executor
#    src/stop1_executor.cpp
#    test/unit_tests/unittest_stop1_executor.cpp
#  )
#  target_link_libraries(unittest_stop1_executor
#    ${catkin_LIBRARIES}
#  )
  #----------------------------------

  #--- StoModbusAdapter intrgration test ---
#  catkin_add_gmock(unittest_modbus_adapter_run_permitted
#    test/unit_tests/unittest_modbus_adapter_run_permitted.cpp
#    src/modbus_adapter_run_permitted.cpp
#    src/modbus_msg_run_permitted_wrapper.cpp
#    src/modbus_msg_in_builder.cpp
#  )
#  target_link_libraries(unittest_modbus_adapter_run_permitted ${catkin_LIBRARIES})
#  add_dependencies(unittest_modbus_adapter_run_permitted ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------

  # --- Stop integration test ---
  add_rostest_gmock(integrationtest_stop1
     test/integration_tests/integrationtest_stop1.test
     test/integration_tests/integrationtest_stop1.cpp
     test/unit_tests/pilz_modbus_server_mock.cpp
     src/libmodbus_client.cpp
     src/modbus_check_ip_connection.cpp
  )
  target_link_libraries(integrationtest_stop1
    ${catkin_LIBRARIES} ${${PROJECT_NAME}_LIBRARIES}
    modbus
  )
  add_dependencies(integrationtest_stop1
#    pilz_modbus_client_node
#    modbus_adapter_run_permitted_node
#    stop1_executor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  #----------------------------------

  # --- Stop1 missing service integration tests ---
  add_rostest_gtest(integrationtest_stop1_service_missing
     test/integration_tests/integrationtest_stop1_service_missing.test
     test/integration_tests/integrationtest_stop1_service_missing.cpp
  )
  target_link_libraries(integrationtest_stop1_service_missing
    ${catkin_LIBRARIES}
  )
  add_dependencies(integrationtest_stop1_service_missing
#    stop1_executor_node
    ${catkin_EXPORTED_TARGETS}
  )

  foreach(service_name halt hold unhold)
    add_rostest(test/integration_tests/integrationtest_stop1_service_missing.test
      ARGS omit_service:=${service_name}
      DEPENDENCIES integrationtest_stop1_service_missing)
  endforeach(service_name)
  #----------------------------------

  # --- Brake test required integration test ---
  add_rostest_gmock(integrationtest_brake_test_required
    test/integration_tests/integrationtest_brake_test_required.test
    test/integration_tests/integrationtest_brake_test_required.cpp
    test/unit_tests/pilz_modbus_server_mock.cpp
    src/libmodbus_client.cpp
    src/modbus_check_ip_connection.cpp
  )
  target_link_libraries(integrationtest_brake_test_required
    ${catkin_LIBRARIES}
    modbus
  )
  add_dependencies(integrationtest_brake_test_required
#    pilz_modbus_client_node
#    modbus_adapter_brake_test_node
    ${catkin_EXPORTED_TARGETS}
  )
  #----------------------------------

  # --- Operation mode integration test ---
  add_rostest_gmock(integrationtest_operation_mode
    test/integration_tests/integrationtest_operation_mode.test
    test/integration_tests/integrationtest_operation_mode.cpp
    test/unit_tests/pilz_modbus_server_mock.cpp
    src/libmodbus_client.cpp
    src/modbus_check_ip_connection.cpp
  )
  target_link_libraries(integrationtest_operation_mode
    ${catkin_LIBRARIES}
    modbus
  )
  add_dependencies(integrationtest_operation_mode
#    pilz_modbus_client_node
    modbus_adapter_operation_mode_node
    ${catkin_EXPORTED_TARGETS}
  )
  #----------------------------------

  # --- Operation mode setup integration test ---
  add_rostest_gmock(integrationtest_operation_mode_setup
    test/integration_tests/integrationtest_operation_mode_setup.test
    test/integration_tests/integrationtest_operation_mode_setup.cpp
  )
  target_link_libraries(integrationtest_operation_mode_setup
    ${catkin_LIBRARIES}
  )
  add_dependencies(integrationtest_operation_mode_setup
    modbus_adapter_operation_mode_node
#    operation_mode_setup_executor_node
    ${catkin_EXPORTED_TARGETS}
  )
  #----------------------------------

  # --- Execute brake test integration test ---
  add_rostest_gmock(integrationtest_execute_brake_test
    test/integration_tests/integrationtest_execute_brake_test.test
    test/integration_tests/integrationtest_execute_brake_test.cpp
    test/unit_tests/pilz_modbus_server_mock.cpp
#    test/unit_tests/canopen_chain_node_mock.cpp
    src/libmodbus_client.cpp
    src/modbus_check_ip_connection.cpp
  )
  target_link_libraries(integrationtest_execute_brake_test
    ${catkin_LIBRARIES} ${pilz_testutils_LIBRARIES}
    modbus
  )
  add_dependencies(integrationtest_execute_brake_test
#    modbus_adapter_brake_test_node
#    brake_test_executor_node
#    canopen_braketest_adapter_node
#    pilz_modbus_client_node
    ${catkin_EXPORTED_TARGETS}
  )
  #----------------------------------------

  # --- FilterPipeline unit test ---
  add_rostest_gtest(unittest_filter_pipeline
    test/unit_tests/unittest_filter_pipeline.test
    test/unit_tests/unittest_filter_pipeline.cpp
  )
  target_link_libraries(unittest_filter_pipeline ${catkin_LIBRARIES})
  add_dependencies(unittest_filter_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------

  #--- ModbusMsgInUtils unit test ---
  catkin_add_gtest(unittest_modbus_msg_in_builder
      test/unit_tests/unittest_modbus_msg_in_builder.cpp
      src/modbus_msg_in_builder.cpp
  )
  target_link_libraries(unittest_modbus_msg_in_builder ${catkin_LIBRARIES})
  add_dependencies(unittest_modbus_msg_in_builder ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------------

  #--- OperationModeSetupExecutor unit test ---
#  catkin_add_gmock(unittest_operation_mode_setup_executor
#      test/unit_tests/unittest_operation_mode_setup_executor.cpp
#      src/operation_mode_setup_executor.cpp
#  )
#  target_link_libraries(unittest_operation_mode_setup_executor ${catkin_LIBRARIES})
#  add_dependencies(unittest_operation_mode_setup_executor ${${PROJECT_NAME}_EXPORTED_TARGETS})
  #----------------------------------------

  #--- FakeSpeedOverride unit test ---
  add_rostest_gtest(unittest_fake_speed_override
      test/unit_tests/unittest_fake_speed_override.test
      test/unit_tests/unittest_fake_speed_override.cpp
  )
  target_link_libraries(unittest_fake_speed_override
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_fake_speed_override
    fake_speed_override_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  #----------------------------------------

  #--- DisableSpeedMonitoring unit test ---
  add_rostest_gmock(unittest_disable_speed_monitoring
      test/unit_tests/unittest_disable_speed_monitoring.test
      test/unit_tests/unittest_disable_speed_monitoring.cpp
  )
  target_link_libraries(unittest_disable_speed_monitoring
    ${catkin_LIBRARIES}
  )
  add_dependencies(unittest_disable_speed_monitoring
    fake_speed_override_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
  #----------------------------------------

  #--- utils unittest ---
  catkin_add_gtest(unittest_utils
    test/unit_tests/unittest_utils.cpp
  )
  #----------------------------------------

  # to run: catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*"
                          "*/BrakeTestErrorCodes.h"
                          "*/ModbusMsgInStamped.h"
                          "*/BrakeTest.h"
                          "*/BrakeTestRequest.h"
                          "*/BrakeTestResponse.h"
                          "*/ModbusRegisterBlock.h"
                          "*/WriteModbusRegister.h"
                          "*/WriteModbusRegisterRequest.h"
                          "*/WriteModbusRegisterResponse.h"
                          "*/SendBrakeTestResult.h"
                          "*/SendBrakeTestResultRequest.h"
                          "*/SendBrakeTestResultResponse.h"
                          "*/FakeSpeedOverrideConfig.h"
                          )
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
