/*
 *  schunk_arm_hardware_interface.h
 *
 *  Schunk 7 DOF arm hardware interface for ros_control
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


#ifndef SCHUNK_ARM_HARDWARE_INTERFACE_H_
#define SCHUNK_ARM_HARDWARE_INTERFACE_H_

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <dumbo_powercube_chain/PowerCubeCtrl.h>
#include <dumbo_powercube_chain/PowerCubeCtrlParams.h>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <pthread.h>
#include <kvaser_canlib/canlib.h>

namespace dumbo_hardware_interface
{

class SchunkArmHardwareInterface : public PowerCubeCtrl
{

public:

    // nodehandle for getting ROS parameters and setting up services
    // CAN mutex and handles in case the arm shares CAN bus with e.g.
    // a gripper
    SchunkArmHardwareInterface(const ros::NodeHandle &nh,
                               boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                               boost::shared_ptr<canHandle> CAN_handle);

    ~SchunkArmHardwareInterface();

    // gets ROS parameters including joint names,
    // CAN parameters, etc.
    void getROSParams();

    // get parameters related to robot joints, i.e.,
    // min/max joint positions, velocities, etc.
    void getRobotDescriptionParams();

    // registers hardware interface handles for joint states and
    // joint velocity commands
    void registerHandles(hardware_interface::JointStateInterface &js_interface,
                         hardware_interface::VelocityJointInterface &vj_interface);

    // connects to arm on CAN bus
    bool connect();

    // disconnects arm from CAN bus
    bool disconnect();

    // read joint positions from encoders
    // reads only one module
    // If wait_for_response==True then it performs a blocking CAN read call
    // (it waits until a msg has arrived)
    // It assumes that a write() call was made in the previous iteration
    void read(bool wait_for_response=false);

    // execute joint velocity command.
    // sends to command to only one joint
    // Does not wait for status msg response, this msg
    // will be read by the read() function
    void write();

    // writes command to a module and waits for its status message
    // reply over the CAN bus
    void writeAndRead();


private:

    // ros stuff
    ros::NodeHandle nh_;

    std::string arm_name_; // "left" or "right"

    // current module being controlled
    // -1: no command has been sent to a module
    // 1-7: a command has been set to this module
    int module_number_;

    // indicates if a write() has been performed, helps for
    // sync of read() and write(). In such case the read()
    // will receive the status response from the module
    bool written_;

    // arm parameters and variables for hardware handles
    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_efforts_;

    std::vector<double> joint_velocity_command_;
};

}


#endif
