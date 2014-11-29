/*
 *  pg70_hw.cpp
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

#include <dumbo_hardware_interface/pg70_hw.h>
#include <urdf/model.h>


namespace dumbo_hardware_interface
{

PG70HW::PG70HW(const ros::NodeHandle &nh,
                                             boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                                             boost::shared_ptr<canHandle> CAN_handle) :
    PG70Gripper(CAN_mutex, CAN_handle),
    nh_(nh)
{
    params_.reset(new PowerCubeCtrlParams());

    getROSParams();
    getRobotDescriptionParams();
}

PG70HW::~PG70HW()
{

}


void PG70HW::getROSParams()
{
    /// get CanBaudrate
    int CanBaudrate;
    if (nh_.hasParam("can_baudrate"))
    {
        nh_.getParam("can_baudrate", CanBaudrate);
    }
    else
    {
        ROS_ERROR("Parameter can_baudrate not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    /// Get arm name parameter (left or right arm)
    XmlRpc::XmlRpcValue ArmNameXmlRpc;
    std::string arm_name;
    if (nh_.hasParam("arm_name"))
    {
        nh_.getParam("arm_name", ArmNameXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter arm_name not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    arm_name = (std::string)(ArmNameXmlRpc);
    if((arm_name!="left") && (arm_name!="right"))
    {
        ROS_ERROR("Invalid arm_name parameter in /PG70_controller, shutting down node... ");
        nh_.shutdown();
    }


    /// get Modul IDs
    XmlRpc::XmlRpcValue ModulIDsXmlRpc;
    std::vector<int> ModulIDs;
    if (nh_.hasParam("module_id"))
    {
        nh_.getParam("module_id", ModulIDsXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter modul_id not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the ModulIDs
    ModulIDs.resize(ModulIDsXmlRpc.size());
    for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
    {
        ModulIDs[i] = (int)ModulIDsXmlRpc[i];
    }

    /// Get joint names
    XmlRpc::XmlRpcValue JointNamesXmlRpc;
    std::vector<std::string> JointNames;
    if (nh_.hasParam("joint_names"))
    {
        nh_.getParam("joint_names", JointNamesXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter joint_names not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
        JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }


    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (nh_.hasParam("max_acceleration"))
    {
        nh_.getParam("max_acceleration", MaxAccelerationsXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter max_acceleration not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
        MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// get Gripper Max acceleration
    int serial_number;
    if (nh_.hasParam("serial_number"))
    {
        nh_.getParam("serial_number", serial_number);
    }

    else
    {
        ROS_ERROR("Parameter serial_number not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    params_->Init(CanBaudrate, ModulIDs);
    params_->setArmName(arm_name);
    params_->SetJointNames(JointNames);
    params_->SetMaxAcc(MaxAccelerations);
    params_->SetSerialNumber((unsigned long int) serial_number);
}


void PG70HW::getRobotDescriptionParams()
{
    /// Get robot_description from ROS parameter server
    std::string param_name = "robot_description";
    std::string full_param_name;
    std::string xml_string;

    nh_.searchParam(param_name, full_param_name);
    if (nh_.hasParam(full_param_name))
    {
        nh_.getParam(full_param_name.c_str(), xml_string);
    }

    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
        nh_.shutdown();
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
        nh_.shutdown();
    }
    ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());

    /// Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh_.shutdown();
    }
    ROS_DEBUG("Successfully parsed urdf file");


    // Get gripper params
    std::vector<std::string> JointNames = params_->GetJointNames();
    std::vector<double> LowerLimits(JointNames.size());
    std::vector<double> UpperLimits(JointNames.size());
    std::vector<double> MaxVel(JointNames.size());
    for(unsigned int i=0; i<JointNames.size(); i++ )
    {
        // Get gripper lower limit
        LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower*2.0;

        // Get gripper upper limit
        UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper*2.0;

        // Get gripper max vel
        MaxVel[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
    }


    params_->SetLowerLimits(LowerLimits);
    params_->SetUpperLimits(UpperLimits);
    params_->SetMaxVel(MaxVel);
}

void PG70HW::registerHandles(hardware_interface::JointStateInterface &js_interface,
                                            hardware_interface::VelocityJointInterface &vj_interface,
                                            hardware_interface::PositionJointInterface &pj_interface)
{
    // initialize joint state and commands
    joint_names_ = params_->GetJointNames();

    unsigned int dof = joint_names_.size();
    joint_positions_ = std::vector<double>(dof, 0.0);
    joint_velocities_ = std::vector<double>(dof, 0.0);
    joint_efforts_ = std::vector<double>(dof, 0.0);

    joint_velocity_command_ = std::vector<double>(dof, 0.0);
    joint_position_command_ = std::vector<double>(dof, 0.0);

    for(unsigned int i=0; i<dof; i++)
    {
        js_interface.registerHandle(hardware_interface::JointStateHandle(
                                        joint_names_[i],
                                        &joint_positions_[i],
                                        &joint_velocities_[i],
                                        &joint_efforts_[i]));


        // Create velocity joint interface
        vj_interface.registerHandle(hardware_interface::JointHandle(
                                        js_interface.getHandle(joint_names_[i]),
                                        &joint_velocity_command_[i]));

        pj_interface.registerHandle(hardware_interface::JointHandle(
                                        js_interface.getHandle(joint_names_[i]),
                                        &joint_position_command_[i]));

    }
}

bool PG70HW::connect()
{
    bool ret = init();

    // reset joint variables
    if(ret)
    {
        unsigned int dof = 2;
        joint_positions_[0] = getPositions()[0]/2.0;
        joint_positions_[1] = getPositions()[0]/2.0;
        joint_velocities_ = std::vector<double>(dof, 0.0);
        joint_efforts_ = std::vector<double>(dof, 0.0);
        return true;
    }

    return false;
}

bool PG70HW::disconnect()
{
    return close();
}

void PG70HW::read()
{
    if(isInitialized())
    {
        // velocity control mode, read feedback status message
        if(!executingPosCommand())
        {
            joint_positions_[0] = getPositions()[0]/2.0;
            joint_positions_[1] = getPositions()[0]/2.0;
            joint_velocities_[0] = getVelocities()[0];
            joint_velocities_[1] = getVelocities()[0];
        }

        // if executing a position command, then read status
        else
        {
            bool ret = updateStates();
            joint_positions_[0] = getPositions()[0]/2.0;
            joint_positions_[1] = getPositions()[0]/2.0;
            joint_velocities_[0] = getVelocities()[0];
            joint_velocities_[1] = getVelocities()[0];
        }
    }

}


void PG70HW::writeReadVel()
{
    // send velocity commands as long as position command is not being executed
    if(isInitialized() && !executingPosCommand())
    {
        bool ret = moveVel(joint_velocity_command_[0]);

        if(!ret)
        {
            ROS_ERROR("Error executing velocity command on PG70 gripper on %s arm", params_->getArmName().c_str());
            return;
        }
    }
}

void PG70HW::writeReadPos()
{
    if(isInitialized())
    {
        // execute position command
        bool ret = movePos(joint_position_command_[0]);

        if (!ret)
        {
            ROS_ERROR("Error executing position command on PG70 gripper", params_->getArmName().c_str());
            return;
        }
    }
}


}
