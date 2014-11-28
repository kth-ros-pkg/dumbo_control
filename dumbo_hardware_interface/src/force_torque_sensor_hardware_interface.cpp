/*
 *  force_torque_sensor_hardware_interface.cpp
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

#include <dumbo_hardware_interface/force_torque_sensor_hardware_interface.h>


namespace dumbo_hardware_interface
{

ForceTorqueSensorHardwareInterface::ForceTorqueSensorHardwareInterface(const ros::NodeHandle &nh) :
    ForceTorqueSensor(),
    nh_(nh), written_(false)
{
    force_.resize(3);
    torque_.resize(3);

    getROSParams();
}

ForceTorqueSensorHardwareInterface::~ForceTorqueSensorHardwareInterface()
{

}

void ForceTorqueSensorHardwareInterface::getROSParams()
{
    std::string serial_number;
    if (nh_.hasParam("serial_number"))
    {
        nh_.getParam("serial_number", serial_number);
    }

    else
    {
        ROS_ERROR("Parameter SerialNumber not available");
        nh_.shutdown();
        return;
    }

    std::string arm_name;
    if (nh_.hasParam("arm_name"))
    {
        nh_.getParam("arm_name", arm_name);
    }

    else
    {
        ROS_ERROR("Parameter arm_name not available");
        nh_.shutdown();
        return;
    }


    serial_number_ = serial_number;
    arm_name_ = arm_name;
}


void ForceTorqueSensorHardwareInterface::registerHandles(hardware_interface::ForceTorqueSensorInterface &ft_interface)
{
    ft_interface.registerHandle(hardware_interface::ForceTorqueSensorHandle(
                                    arm_name_+"_arm_ft_sensor",
                                    arm_name_+"_arm_ft_sensor",
                                    &(force_[0]),
                                    &(torque_[0])));
}

bool ForceTorqueSensorHardwareInterface::connect()
{
    bool ret = init(serial_number_, arm_name_);

    if(ret)
    {
        written_ = false;
    }

    return ret;
}


void ForceTorqueSensorHardwareInterface::read()
{
    if(isInitialized() && written_)
    {
        bool ret = readFT(force_, torque_);

        if(!ret)
        {
            ROS_ERROR("Error reading %s arm FT sensor", arm_name_.c_str());
        }
    }
}


void ForceTorqueSensorHardwareInterface::write()
{
    if(isInitialized())
    {
        bool ret = requestFT();

        if(ret)
        {
            written_ = true;
        }

        else
        {
            ROS_ERROR("Error requesting FT measurement %s arm", arm_name_.c_str());
        }

    }

}

}
