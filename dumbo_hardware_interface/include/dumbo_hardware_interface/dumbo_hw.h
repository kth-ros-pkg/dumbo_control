/*
 *  dumbo_hw.h
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


#ifndef DUMBO_HARDWARE_H_
#define DUMBO_HARDWARE_H_

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include <dumbo_hardware_interface/schunk_arm_hw.h>
#include <dumbo_hardware_interface/pg70_hw.h>
#include <dumbo_hardware_interface/force_torque_sensor_hw.h>

#include <boost/scoped_ptr.hpp>

namespace dumbo_hardware_interface
{

class DumboHW : public hardware_interface::RobotHW
{

public:

    boost::scoped_ptr<SchunkArmHW> left_arm_hw;
    boost::scoped_ptr<SchunkArmHW> right_arm_hw;

    boost::scoped_ptr<ForceTorqueSensorHW> left_ft_sensor_hw;
    boost::scoped_ptr<ForceTorqueSensorHW> right_ft_sensor_hw;

    boost::scoped_ptr<PG70HW> pg70_hw;

    DumboHW();

    ~DumboHW();

    bool connect();

    void disconnect();

    void stop();

    void recover();

    // read arms, gripper, ft sensors
    void read();

    // sends velocity commands to arms
    // requests ft measurements
    // if PG70 is in velocity mode
    // send velocity commands to it
    void write();

    // sends velocity commands to arms
    // requests ft measurements
    // send a position command to PG70
    // in this case do read&write in corresponding schunk arm
    void write(double gripper_pos_command);


private:

    //hardware interfaces
    hardware_interface::JointStateInterface js_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::ForceTorqueSensorInterface ft_sensor_interface_;

    // register HW handles and interfaces for ros_control
    void registerHW();

};


}


#endif
