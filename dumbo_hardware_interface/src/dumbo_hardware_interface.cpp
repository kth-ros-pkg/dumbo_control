/*
 *  dumbo_hardware_interface.cpp
 *
 *  Dumbo hardware interface for the ros_control framework
 *  Created on: Nov 20, 2014
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

#include <dumbo_hardware_interface/dumbo_hardware_interface.h>


namespace dumbo_hardware_interface
{
DumboHardwareInterface::DumboHardwareInterface()
{

    boost::shared_ptr<pthread_mutex_t> left_arm_CAN_mutex(new pthread_mutex_t);
    *left_arm_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;

    boost::shared_ptr<canHandle> left_arm_CAN_handle(new canHandle(0));

    left_arm_hw_.reset(new SchunkArmHardwareInterface(ros::NodeHandle("/left_arm"),
                                                      left_arm_CAN_mutex,
                                                      left_arm_CAN_handle));

    pg70_hw_.reset(new PG70HardwareInterface(ros::NodeHandle("/PG70_gripper"),
                                             left_arm_CAN_mutex,
                                             left_arm_CAN_handle));

    boost::shared_ptr<pthread_mutex_t> right_arm_CAN_mutex(new pthread_mutex_t);

    *right_arm_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;
    boost::shared_ptr<canHandle> right_arm_CAN_handle(new canHandle(0));

    right_arm_hw_.reset(new SchunkArmHardwareInterface(ros::NodeHandle("/right_arm"),
                                                      right_arm_CAN_mutex,
                                                      right_arm_CAN_handle));

    left_ft_sensor_hw_.reset(new ForceTorqueSensorHardwareInterface(ros::NodeHandle("/left_arm_ft_sensor")));
    right_ft_sensor_hw_.reset(new ForceTorqueSensorHardwareInterface(ros::NodeHandle("/right_arm_ft_sensor")));

    // register hardware interfaces
    registerHW();
}


bool DumboHardwareInterface::connect()
{
    if(!left_arm_hw_->connect())
    {
        ROS_ERROR("Error connecting to left arm");
        return false;
    }

    if(!right_arm_hw_->connect())
    {
        ROS_ERROR("Error connecting to right arm");
        return false;
    }

    if(!pg70_hw_->connect())
    {
        ROS_ERROR("Error connecting to PG70 gripper");
        return false;
    }

    if(!left_ft_sensor_hw_->connect())
    {
        ROS_ERROR("Error connecting left arm FT sensor");
        return false;
    }

    if(!right_ft_sensor_hw_->connect())
    {
        ROS_ERROR("Error connecting right arm FT sensor");
        return false;
    }
}


void DumboHardwareInterface::disconnect()
{
    left_arm_hw_->disconnect();
    right_arm_hw_->disconnect();
    pg70_hw_->disconnect();
    left_ft_sensor_hw_->disconnect();
    right_ft_sensor_hw_->disconnect();
}


void DumboHardwareInterface::stop()
{
    left_arm_hw_->stop();
    right_arm_hw_->stop();
}

void DumboHardwareInterface::recover()
{
    left_arm_hw_->recover();
    right_arm_hw_->recover();
    pg70_hw_->recover();
}

void DumboHardwareInterface::read()
{
    left_arm_hw_->read();
    right_arm_hw_->read();

    pg70_hw_->read();

    left_ft_sensor_hw_->read();
    right_ft_sensor_hw_->read();
}

void DumboHardwareInterface::write()
{
    pg70_hw_->writeReadVel();

    left_arm_hw_->write();
    right_arm_hw_->write();

    left_ft_sensor_hw_->write();
    right_ft_sensor_hw_->write();
}

void DumboHardwareInterface::write(double gripper_pos_command)
{
    // set gripper pos command on hardware interface
    hardware_interface::JointHandle pg70_joint_handle = pj_interface_.getHandle("left_arm_top_finger_joint");
    hardware_interface::JointHandle pg70_joint_handle2 = pj_interface_.getHandle("left_arm_bottom_finger_joint");

    pg70_joint_handle.setCommand(gripper_pos_command);
    pg70_joint_handle2.setCommand(gripper_pos_command);

    // send position command to PG70
    pg70_hw_->writeReadPos();

    left_arm_hw_->write();
    right_arm_hw_->write();

    left_ft_sensor_hw_->write();
    right_ft_sensor_hw_->write();
}

void DumboHardwareInterface::registerHW()
{
    // register the handles
    left_arm_hw_->registerHandles(js_interface_,
                                  vj_interface_);
    right_arm_hw_->registerHandles(js_interface_,
                                   vj_interface_);

    pg70_hw_->registerHandles(js_interface_,
                              vj_interface_,
                              pj_interface_);

    left_ft_sensor_hw_->registerHandles(ft_sensor_interface_);
    right_ft_sensor_hw_->registerHandles(ft_sensor_interface_);


    // register the interfaces
    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&ft_sensor_interface_);
}

}
