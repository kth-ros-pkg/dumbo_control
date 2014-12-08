/*
 *  dumbo_hw.cpp
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

#include <dumbo_hardware_interface/dumbo_hw.h>


namespace dumbo_hardware_interface
{
DumboHW::DumboHW()
{

    boost::shared_ptr<pthread_mutex_t> left_arm_CAN_mutex(new pthread_mutex_t);
    *left_arm_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;

    boost::shared_ptr<canHandle> left_arm_CAN_handle(new canHandle(0));

    left_arm_hw.reset(new SchunkArmHW(ros::NodeHandle("/left_arm"),
                                                      left_arm_CAN_mutex,
                                                      left_arm_CAN_handle));

    pg70_hw.reset(new PG70HW(ros::NodeHandle("/PG70_gripper"),
                                             left_arm_CAN_mutex,
                                             left_arm_CAN_handle));

    boost::shared_ptr<pthread_mutex_t> right_arm_CAN_mutex(new pthread_mutex_t);

    *right_arm_CAN_mutex = PTHREAD_MUTEX_INITIALIZER;
    boost::shared_ptr<canHandle> right_arm_CAN_handle(new canHandle(0));

    right_arm_hw.reset(new SchunkArmHW(ros::NodeHandle("/right_arm"),
                                                      right_arm_CAN_mutex,
                                                      right_arm_CAN_handle));

    left_ft_sensor_hw.reset(new ForceTorqueSensorHW(ros::NodeHandle("/left_arm_ft_sensor")));
    right_ft_sensor_hw.reset(new ForceTorqueSensorHW(ros::NodeHandle("/right_arm_ft_sensor")));

    // register hardware interfaces
    registerHW();
}

DumboHW::~DumboHW()
{

}


void DumboHW::connect()
{
    if(!left_arm_hw->connect())
    {
        ROS_ERROR("Error connecting to left arm");
    }

    if(!right_arm_hw->connect())
    {
        ROS_ERROR("Error connecting to right arm");
    }

    if(!pg70_hw->connect())
    {
        ROS_ERROR("Error connecting to PG70 gripper");
    }

    if(!left_ft_sensor_hw->connect())
    {
        ROS_ERROR("Error connecting left arm FT sensor");
    }

    if(!right_ft_sensor_hw->connect())
    {
        ROS_ERROR("Error connecting right arm FT sensor");
    }

}


void DumboHW::disconnect()
{
    left_arm_hw->disconnect();
    right_arm_hw->disconnect();
    pg70_hw->disconnect();
    left_ft_sensor_hw->disconnect();
    right_ft_sensor_hw->disconnect();
}


void DumboHW::stop()
{
    left_arm_hw->stop();
    right_arm_hw->stop();
}

void DumboHW::recover()
{
    left_arm_hw->recover();
    right_arm_hw->recover();
    pg70_hw->recover();
}

void DumboHW::read()
{
    left_arm_hw->read(true);
    right_arm_hw->read(true);

    pg70_hw->read();

    left_ft_sensor_hw->read();
    right_ft_sensor_hw->read();
}

void DumboHW::write()
{
    pg70_hw->writeReadVel();

    left_arm_hw->write();
    right_arm_hw->write();

    left_ft_sensor_hw->write();
    right_ft_sensor_hw->write();
}

void DumboHW::write(double gripper_pos_command)
{

    // send position command to PG70
    pg70_hw->writeReadPos(gripper_pos_command);

    left_arm_hw->write();
    right_arm_hw->write();

    left_ft_sensor_hw->write();
    right_ft_sensor_hw->write();
}

void DumboHW::registerHW()
{
    // register the handles
    left_arm_hw->registerHandles(js_interface_,
                                  vj_interface_);
    right_arm_hw->registerHandles(js_interface_,
                                   vj_interface_);

    pg70_hw->registerHandles(js_interface_,
                              vj_interface_,
                              pj_interface_);

    left_ft_sensor_hw->registerHandles(ft_sensor_interface_);
    right_ft_sensor_hw->registerHandles(ft_sensor_interface_);


    // register the interfaces
    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&ft_sensor_interface_);
}

}
