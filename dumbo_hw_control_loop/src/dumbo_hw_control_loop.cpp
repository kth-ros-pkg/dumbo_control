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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <control_msgs/GripperCommand.h>
#include <boost/scoped_ptr.hpp>

namespace dumbo_control
{
class DumboHWControlLoop
{
public:

    // thread variables for realtime control loop
    pthread_t controlThread;
    pthread_attr_t controlThreadAttr;

    DumboHWControlLoop():
        connect_dumbo_(false),
        disconnect_dumbo_(false),
        stop_dumbo_(false),
        recover_dumbo_(false),
        connect_left_arm_(false),
        disconnect_left_arm_(false),
        connect_right_arm_(false),
        disconnect_right_arm_(false),
        connect_pg70_(false),
        disconnect_pg70_(false)
    {
        nh_ = ros::NodeHandle("~");

        advertiseServices();
        advertiseTopics();

        // subscribe to gripper pos command
        pg70_pos_command_sub_ = nh_.advertise("/PG70_gripper/pos_command");

        // start realtime hw control loop thread
        int rv;
        if ((rv = pthread_create(&controlThread, &controlThreadAttr, DumboHWControlLoop::controlLoop, 0)) != 0)
        {
          ROS_FATAL("Unable to create control thread: rv = %d", rv);
          exit(EXIT_FAILURE);
        }

    }

    ~DumboHWControlLoop()
    {
    }

    void advertiseServices()
    {

        connect_dumbo_srv_server_ = nh_.advertiseService("/dumbo/connect", &DumboHWControlLoop::connectDumboSrvCallback, this);
        disconnect_dumbo_srv_server_ = nh_.advertiseService("/dumbo/disconnect", &DumboHWControlLoop::disconnectDumboSrvCallback, this);

        stop_dumbo_srv_server_ = nh_.advertiseService("/dumbo/stop", &DumboHWControlLoop::stopDumboSrvCallback, this);
        recover_dumbo_srv_server_ = nh_.advertiseService("/dumbo/recover", &DumboHWControlLoop::recoverDumboSrvCallback, this);

        connect_left_arm_srv_server_ = nh_.advertiseService("/left_arm/connect", &DumboHWControlLoop::connectLeftArmSrvCallback, this);
        disconnect_left_arm_srv_server_ = nh_.advertiseService("/left_arm/disconnect", &DumboHWControlLoop::disconnectLeftArmSrvCallback, this);

        connect_right_arm_srv_server_ = nh_.advertiseService("/right_arm/connect", &DumboHWControlLoop::connectRightArmSrvCallback, this);
        disconnect_right_arm_srv_server_ = nh_.advertiseService("/right_arm/disconnect", &DumboHWControlLoop::disconnectRightArmSrvCallback, this);

        connect_pg70_srv_server_ = nh_.advertiseService("/PG70_gripper/connect", &DumboHWControlLoop::connectPG70SrvCallback, this);
        disconnect_pg70_srv_server_ = nh_.advertiseService("/PG70_gripper/disconnect", &DumboHWControlLoop::disconnectPG70SrvCallback, this);
    }

    void advertiseTopics()
    {
        left_arm_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/left_arm/connected", 1));
        right_arm_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/right_arm/connected", 1));

        pg70_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/PG70_gripper/connected", 1));

        left_arm_ft_sensor_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/left_arm_ft_sensor/connected", 1));
        right_arm_ft_sensor_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/right_arm_ft_sensor/connected", 1));
    }

    // service callback definitions

    bool connectDumboSrvCallback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        connect_dumbo_ = true;
    }

    bool disconnectDumboSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        disconnect_dumbo_ = true;
    }

    bool stopDumboSrvCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res)
    {
        stop_dumbo_ = true;
    }

    bool recoverDumboSrvCallback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        recover_dumbo_ = true;
    }

    bool connectLeftArmSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        connect_left_arm_ = true;
    }

    bool disconnectLeftArmSrvCallback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
    {
        disconnect_left_arm_ = true;
    }


    bool connectRightArmSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        connect_right_arm_ = true;
    }

    bool disconnectRightArmSrvCallback(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res)
    {
        disconnect_right_arm_ = true;
    }

    bool connectPG70SrvCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res)
    {
        connect_pg70_ = true;
    }

    bool disconnectPG70SrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        disconnect_pg70_ = true;
    }

    void pg70PosCommandCallback(control_msgs::GripperCommand::ConstPtr &gripper_pos_command)
    {
        pg70_pos_command_requested_ = true;
        pg70_pos_command_ = gripper_pos_command->position;
    }

    void controlLoop()
    {

        ros::Time last, now;
        ros::Duration period(1.0);

        ros::NodeHandle nh;
        controller_manager::ControllerManager cm(&dumbo_hw_, nh);

        while(nh.ok())
        {
            int count = 0;

            if(count++>1000)
            {
                count = 0;
                // attend service requests (connect, disconnect, stop, recover)
                // only done every 1000 iterations
                attendServiceRequests();

                // publish hw status message with realtime publishers
                // but only every 100 iterations of the control loop
                publishHWStatus();
            }

            dumbo_hw_.read();

            cm.update(now, period);

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
        if(connect_dumbo_)
        {
            dumbo_hw_.connect();
            connect_dumbo_ = false;
        }

        else if(disconnect_dumbo_)
        {
            dumbo_hw_.disconnect();
            disconnect_dumbo_ = false;
        }

        if(stop_dumbo_)
        {
            dumbo_hw_.stop();
            stop_dumbo_ = false;
        }

        else if(recover_dumbo_)
        {
            dumbo_hw_.recover();
            recover_dumbo_ = false;
        }

        if(connect_left_arm_)
        {
            dumbo_hw_.left_arm_hw->connect();
            connect_left_arm_ = false;
        }

        else if(disconnect_left_arm_)
        {
            dumbo_hw_.left_arm_hw->disconnect();
            disconnect_left_arm_ = false;
        }

        if(connect_right_arm_)
        {
            dumbo_hw_.right_arm_hw->connect();
            connect_right_arm_ = false;
        }

        else if(disconnect_right_arm_)
        {
            dumbo_hw_.right_arm_hw->disconnect();
            disconnect_right_arm_ = false;
        }

        if(connect_pg70_)
        {
            dumbo_hw_.pg70_hw->connect();
            connect_pg70_ = false;
        }

        else if(disconnect_pg70_)
        {
            dumbo_hw_.pg70_hw->disconnect();
            disconnect_pg70_ = false;
        }
    }

    void publishHWStatus()
    {

        if(left_arm_status_publisher_->trylock())
        {
            left_arm_status_publisher_->msg_.data = dumbo_hw_.left_arm_hw->isInitialized();
            left_arm_status_publisher_->unlockAndPublish();
        }

        if(right_arm_status_publisher_->trylock())
        {
            right_arm_status_publisher_->msg_.data = dumbo_hw_.right_arm_hw->isInitialized();
            right_arm_status_publisher_->unlockAndPublish();
        }

        if(pg70_status_publisher_->trylock())
        {
            pg70_status_publisher_->msg_.data = dumbo_hw_.pg70_hw->isInitialized();
            pg70_status_publisher_->unlockAndPublish();
        }

        if(left_arm_ft_sensor_status_publisher_->trylock())
        {
            left_arm_ft_sensor_status_publisher_->msg_.data = dumbo_hw_.left_ft_sensor_hw->isInitialized();
            left_arm_ft_sensor_status_publisher_->unlockAndPublish();
        }

        if(right_arm_ft_sensor_status_publisher_->trylock())
        {
            right_arm_ft_sensor_status_publisher_->msg_.data = dumbo_hw_.right_ft_sensor_hw->isInitialized();
            right_arm_ft_sensor_status_publisher_->unlockAndPublish();
        }
    }

private:
    ros::NodeHandle nh_;
    dumbo_hardware_interface::DumboHW dumbo_hw_;

    // services for connecting/disconnecting to hw
    // as well as stopping/recovering manipulators
    ros::ServiceServer connect_dumbo_srv_server_;
    ros::ServiceServer disconnect_dumbo_srv_server_;

    ros::ServiceServer stop_dumbo_srv_server_;
    ros::ServiceServer recover_dumbo_srv_server_;

    ros::ServiceServer connect_left_arm_srv_server_;
    ros::ServiceServer disconnect_left_arm_srv_server_;

    ros::ServiceServer connect_right_arm_srv_server_;
    ros::ServiceServer disconnect_right_arm_srv_server_;

    ros::ServiceServer connect_pg70_srv_server_;
    ros::ServiceServer disconnect_pg70_srv_server_;

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

    // publishers for hw state (connected = True or False)
    boost::scoped_ptr<realtime_tools::RealtimePublisher> left_arm_status_publisher_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher> right_arm_status_publisher_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher> pg70_status_publisher_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher> left_arm_ft_sensor_status_publisher_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher> right_arm_ft_sensor_status_publisher_;

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

    // creates the realtime hw control loop
    dumbo_control::DumboHWControlLoop dumbo_hw_control_loop;

    ros::spin();

    int rv;
    pthread_joint(dumbo_hw_control_loop.controlThread, (void **)&rv);

    return 0;
}
