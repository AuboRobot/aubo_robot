#include "aubo_driver/aubo_driver.h"

#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

#include "ros/ros.h"
#include <ros/console.h>
#include <aubo_msgs/SetIO.h>
#include <aubo_msgs/SetPayload.h>
#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>
#include <aubo_msgs/IOState.h>
#include <aubo_msgs/Digital.h>
#include <aubo_msgs/Analog.h>
#include "std_msgs/String.h"

namespace aubo_driver
{
    class AuboIOControl
    {
    protected:
        AuboDriver robot_;
        ros::NodeHandle nh_;
        ros::ServiceServer io_srv_;
        ros::ServiceServer payload_srv_;
        double io_flag_delay_;
        double max_velocity_;

    public:
        bool setIO(aubo_msgs::SetIORequest& req, aubo_msgs::SetIOResponse& resp)
        {
            resp.success = true;
            //if (req.fun == ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT) {
            if (req.fun == 1)
            {
                robot_.setDigitalOut(req.pin, req.state > 0.0 ? true : false);
            }
            else if (req.fun == 2)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_FLAG) {
                robot_.setFlag(req.pin, req.state > 0.0 ? true : false);
                //According to urdriver.py, set_flag will fail if called to rapidly in succession
                ros::Duration(io_flag_delay_).sleep();
            }
            else if (req.fun == 3)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_ANALOG_OUT) {
                robot_.setAnalogOut(req.pin, req.state);
            }
            else if (req.fun == 4)
            {
                //} else if (req.fun == ur_msgs::SetIO::Request::FUN_SET_TOOL_VOLTAGE) {
                robot_.setToolVoltage((int) req.state);
            }
            else
            {
                resp.success = false;
            }
            return resp.success;
        }

        bool setPayload(aubo_msgs::SetPayloadRequest& req, aubo_msgs::SetPayloadResponse& resp)
        {
            if (robot_.setPayload(req.payload))
                resp.success = true;
            else
                resp.success = true;
            return resp.success;
        }

        void publishMbMsg()
        {
            bool warned = false;
            ros::Publisher io_pub = nh_.advertise<ur_msgs::IOStates>(
                        "ur_driver/io_states", 1);

            while (ros::ok()) {
                ur_msgs::IOStates io_msg;
                std::mutex msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
                std::unique_lock<std::mutex> locker(msg_lock);
                while (!robot_.sec_interface_->robot_state_->getNewDataAvailable()) {
                    msg_cond_.wait(locker);
                }
                int i_max = 10;
                if (robot_.sec_interface_->robot_state_->getVersion() > 3.0)
                    i_max = 18; // From version 3.0, there are up to 18 inputs and outputs
                for (unsigned int i = 0; i < i_max; i++) {
                    ur_msgs::Digital digi;
                    digi.pin = i;
                    digi.state =
                            ((robot_.sec_interface_->robot_state_->getDigitalInputBits()
                              & (1 << i)) >> i);
                    io_msg.digital_in_states.push_back(digi);
                    digi.state =
                            ((robot_.sec_interface_->robot_state_->getDigitalOutputBits()
                              & (1 << i)) >> i);
                    io_msg.digital_out_states.push_back(digi);
                }
                ur_msgs::Analog ana;
                ana.pin = 0;
                ana.state = robot_.sec_interface_->robot_state_->getAnalogInput0();
                io_msg.analog_in_states.push_back(ana);
                ana.pin = 1;
                ana.state = robot_.sec_interface_->robot_state_->getAnalogInput1();
                io_msg.analog_in_states.push_back(ana);

                ana.pin = 0;
                ana.state = robot_.sec_interface_->robot_state_->getAnalogOutput0();
                io_msg.analog_out_states.push_back(ana);
                ana.pin = 1;
                ana.state = robot_.sec_interface_->robot_state_->getAnalogOutput1();
                io_msg.analog_out_states.push_back(ana);
                io_pub.publish(io_msg);

                if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
                        or robot_.sec_interface_->robot_state_->isProtectiveStopped()) {
                    if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
                            and !warned) {
                        print_error("Emergency stop pressed!");
                    } else if (robot_.sec_interface_->robot_state_->isProtectiveStopped()
                               and !warned) {
                        print_error("Robot is protective stopped!");
                    }
                    if (has_goal_) {
                        print_error("Aborting trajectory");
                        robot_.stopTraj();
                        result_.error_code = result_.SUCCESSFUL;
                        result_.error_string = "Robot was halted";
                        goal_handle_.setAborted(result_, result_.error_string);
                        has_goal_ = false;
                    }
                    warned = true;
                } else
                    warned = false;

                robot_.sec_interface_->robot_state_->finishedReading();
            }
        }


    };

}

