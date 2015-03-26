/*
 *  pg70_hw.h
 *
 *  Schunk PG70 parallel gripper hardware interface for ros_control
 *  Created on: Nov 21, 2014
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


#ifndef PG70_HW_H_
#define PG70_HW_H_

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <dumbo_powercube_chain/PG70Gripper.h>
#include <dumbo_powercube_chain/PowerCubeCtrlParams.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <kvaser_canlib/canlib.h>
#include <cob_srvs/Trigger.h>

namespace dumbo_hardware_interface
{

class PG70HW : public PG70Gripper
{

public:
    PG70HW(const ros::NodeHandle &nh,
           boost::shared_ptr<pthread_mutex_t> CAN_mutex,
           boost::shared_ptr<canHandle> CAN_handle);

    ~PG70HW();

    void getROSParams();

    void getRobotDescriptionParams();

    void registerHandles(hardware_interface::JointStateInterface &js_interface,
                         hardware_interface::VelocityJointInterface &vj_interface,
                         hardware_interface::PositionJointInterface &pj_interface);

    // connects to CAN bus (assumes schunk arm has been
    // connected first
    bool connect();

    // disconnects from CAN bus
    bool disconnect();

    // read joint positions from encoders
    void read();

    // execute joint velocity command
    // reads status feedback message
    void writeReadVel();

    // execute joint position command
    // reads status feedback message
    void writeReadPos(double gripper_pos_command);

    // sends zero velocity command to gripper
    void writeZeroVel();


private:


    // ros stuff
    ros::NodeHandle nh_; // should be in "PG70_controller" namespace

    // arm parameters and variables for hardware handles
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;

    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_position_command_;

};

}



#endif
