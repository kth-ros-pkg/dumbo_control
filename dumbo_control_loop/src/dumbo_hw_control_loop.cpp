/*
 *  dumbo_control_loop.cpp
 *
 *  Runs Dumbo's main low level hardware control loop (Schunk arms, parallel gripper and force-torque sensors) following the ros_control framework.
 *  Created on: Nov 29, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dumbo_hardware_interface/dumbo_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

namespace dumbo_control
{
class DumboHWControlLoop
{
public:

    DumboHWControlLoop()
    {
        nh_ = ros::NodeHandle("~");
        registerServices();

        // start realtime thread loop
    }

    ~DumboHWControlLoop()
    {
    }

    void registerServices()
    {

    }

    // service callback definitions

    bool dumboConnectSrvCallback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        connect_dumbo_ = true;
    }

    bool dumboDisconnectSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        disconnect_dumbo_ = true;
    }

    bool leftArmConnectSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        connect_left_arm_ = true;
    }

    bool leftArmDisconnectSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        disconnect_left_arm_ = true;
    }


    bool rightArmConnectSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        connect_right_arm_ = true;
    }

    bool rightArmDisconnectSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        disconnect_right_arm_ = true;
    }

    bool pg70ConnectSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        connect_pg70_ = true;
    }

    bool pg70DisconnectSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        disconnect_pg70_ = true;
    }

    void controlLoop()
    {

        ros::NodeHandle nh;
        controller_manager::ControllerManager cm(&dumbo_hw_, nh);

        while(nh.ok())
        {
            // attend service requests (connect, disconnect, stop, recover)
            attendServiceRequests();

            // publish hw status message with realtime publishers
            // but only every N iterations of the control loop
            publishHWStatus();

            dumbo_hw_.read();

            cm.update();

            // if a parallel gripper command has been requested
            // then send the command to the gripper
            if(pg70_pos_command_requested_)
            {
                dumbo_hw_.write(pg70_pos_command_);
                pg70_pos_command_requested_ = false;
            }

            // if no PG70 pos command, then execute
            // velocity command on arms & gripper
            else
            {
                dumbo_hw_.write();
            }

            // do some sleep here
        }
    }

    void attendServiceRequests()
    {

    }

    void publishHWStatus()
    {

    }

private:
    ros::NodeHandle nh_;
    dumbo_hardware_interface::DumboHW dumbo_hw_;

    // services for connecting/disconnecting to hw
    // as well as stopping/recovering manipulators
    ros::ServiceServer dumbo_connect_srv_server_;
    ros::ServiceServer dumbo_disconnect_srv_server_;

    ros::ServiceServer dumbo_stop_srv_server_;
    ros::ServiceServer dumbo_recover_srv_server_;

    ros::ServiceServer left_arm_connect_srv_server_;
    ros::ServiceServer left_arm_disconnect_srv_server_;

    ros::ServiceServer right_arm_connect_srv_server_;
    ros::ServiceServer right_arm_disconnect_srv_server_;

    ros::ServiceServer pg70_connect_srv_server_;
    ros::ServiceServer pg70_disconnect_srv_server_;

    bool connect_dumbo_;
    bool disconnect_dumbo_;

    bool stop_dumbo_;
    bool recover_dumbo_;

    bool connect_left_arm_;
    bool disconnect_left_arm_;

    bool connect_right_arm_;
    bool disconnect_right_arm_;

    bool connect_pg70_;
    bool disconnect_pg70_;

    // publishers for hw state
    // they all publish bools or strings
    realtime_tools::RealtimePublisher left_arm_status_publisher_;
    realtime_tools::RealtimePublisher right_arm_status_publisher_;

    realtime_tools::RealtimePublisher pg70_status_publisher_;

    realtime_tools::RealtimePublisher left_arm_ft_sensor_status_publisher_;
    realtime_tools::RealtimePublisher right_arm_ft_sensor_status_publisher_;

    // subscriber for parallel gripper commands
    // todo: place this is as a separate action controller
    ros::Subscriber pg70_pos_command_sub_;

    bool pg70_pos_command_requested_;
    double pg70_pos_command_;

};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dumbo_hw_control_loop");

    dumbo_control::DumboHWControlLoop dumbo_hw_control_loop;

    ros::spin();

    return 0;
}
