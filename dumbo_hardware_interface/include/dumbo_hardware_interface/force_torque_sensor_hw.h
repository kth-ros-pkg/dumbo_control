/*
 *  force_torque_sensor_hw.h
 *
 *  Dumbo's ATI 6-axis force-torque sensors hardware interface for ros_control
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

#ifndef FORCE_TORQUE_SENSOR_HW_H_
#define FORCE_TORQUE_SENSOR_HW_H_


#include <ros/ros.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <dumbo_force_torque_sensor/ForceTorqueSensor.h>
#include <boost/scoped_ptr.hpp>

namespace dumbo_hardware_interface
{

class ForceTorqueSensorHW : public ForceTorqueSensor
{

public:

    ForceTorqueSensorHW(const ros::NodeHandle &nh);

    ~ForceTorqueSensorHW();

    void getROSParams();

    // register force-torque sensor hardware interface handle for ros_control
    void registerHandles(hardware_interface::ForceTorqueSensorInterface &ft_sensor_interface);

    // connects to F/T sensor on CAN bus
    bool connect();

    // read force-torque measurement from CAN bus
    void read();

    // request new force-torque measurement via CAN bus
    void write();


private:
    ros::NodeHandle nh_;

    // buffers for force/torque
    std::vector<double> force_;
    std::vector<double> torque_;

    std::string serial_number_;
    std::string arm_name_;

    bool written_;
};

}


#endif
