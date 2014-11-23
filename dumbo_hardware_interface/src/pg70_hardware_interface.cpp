/*
 *  pg70_hardware_interface.cpp
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

#include <dumbo_hardware_interface/pg70_hardware_interface.h>
#include <urdf/model.h>


namespace dumbo_hardware_interface
{

PG70HardwareInterface::PG70HardwareInterface(const std::string &gripper_name,
                                             const ros::NodeHandle &nh,
                                             boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                                             boost::shared_ptr<canHandle> CAN_handle) :
    gripper_name_(gripper_name),
    nh_(nh),
    connected_(false)
{
    pg70_params_.reset(new PowerCubeCtrlParams());
    pg70_ctrl_.reset(new PowerCubeCtrl(pg70_params_, CAN_mutex, CAN_handle));

    connect_service_server_ = nh_.advertiseService("connect", &PG70HardwareInterface::connectSrvCallback, this);
    disconnect_service_server_ = nh_.advertiseService("disconnect", &PG70HardwareInterface::disconnectSrvCallback, this);
    recover_service_server_ = nh_.advertiseService("recover", &PG70HardwareInterface::recoverSrvCallback, this);

    getROSParams();
    getRobotDescriptionParams();
}

PG70HardwareInterface::~PG70HardwareInterface()
{

}


void PG70HardwareInterface::getROSParams()
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
    int SerialNumber;
    if (nh_.hasParam("serial_number"))
    {
        nh_.getParam("serial_number", SerialNumber);
    }

    else
    {
        ROS_ERROR("Parameter serial_number not set in /PG70_controller, shutting down node...");
        nh_.shutdown();
    }

    pg70_params_->Init(CanBaudrate, ModulIDs);
    pg70_params_->setArmName(arm_name);
    pg70_params_->SetJointNames(JointNames);
    pg70_params_->SetMaxAcc(MaxAccelerations);
    pg70_params_->SetSerialNumber((unsigned long int) SerialNumber);

    // initialize joint state messages
    // TODO: initialize interfaces
}


void PG70HardwareInterface::getRobotDescriptionParams()
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
    std::vector<std::string> JointNames = pg70_params_->GetJointNames();
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


    pg70_params_->SetLowerLimits(LowerLimits);
    pg70_params_->SetUpperLimits(UpperLimits);
    pg70_params_->SetMaxVel(MaxVel);
}

void PG70HardwareInterface::registerHandles(hardware_interface::JointStateInterface &js_interface,
                                            hardware_interface::VelocityJointInterface &vj_interface,
                                            hardware_interface::PositionJointInterface &pj_interface)
{
    // initialize joint state and commands
    unsigned int dof = 2;
    joint_names_ = pg70_params_->GetJointNames();
    joint_positions_ = std::vector<double>(dof, 0.0);
    joint_velocities_ = std::vector<double>(dof, 0.0);
    joint_efforts_ = std::vector<double>(dof, 0.0);

    joint_velocity_command_ = std::vector<double>(dof, 0.0);

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

void PG70HardwareInterface::read()
{

}


void PG70HardwareInterface::write()
{

}

bool PG70HardwareInterface::connectSrvCallback(cob_srvs::Trigger::Request &req,
                                               cob_srvs::Trigger::Response &res)
{
    if (!connected_)
    {
        ROS_INFO("Connecting to PG70 gripper...");

        if(pg70_ctrl_->init())
        {
            connected_ = true;
            res.success.data = true;
            ROS_INFO("...initializing PG70 gripper successful");
        }

        else
        {
            ROS_ERROR("Couldn't initialize PG70 gripper on %s arm.", pg70_params_->getArmName().c_str());
            connected_ = false;
            res.success.data = false;
        }

    }

    else
    {
        res.success.data = true;
        res.error_message.data = "PG70 gripper already initialized";
        ROS_WARN("...initializing PG70 gripper not successful. error: %s",res.error_message.data.c_str());
    }

    return true;
}

bool PG70HardwareInterface::disconnectSrvCallback(cob_srvs::Trigger::Request &req,
                                                  cob_srvs::Trigger::Response &res)
{
    if (!connected_)
    {
        ROS_WARN("PG70 gripper already switched off");
        res.success.data = false;
        res.error_message.data = "PG70 gripper already switched off";
    }

    else
    {
        connected_ = false;
        res.success.data = true;
        ROS_INFO("Shutting down PG70 gripper");
    }


    return true;
}

bool PG70HardwareInterface::recoverSrvCallback(cob_srvs::Trigger::Request &req,
                                               cob_srvs::Trigger::Response &res)
{
    ROS_INFO("Recovering PG70 Gripper...");
    if (connected_)
    {
        /// stopping all arm movements
        if (pg70_ctrl_->Recover())
        {
            res.success.data = true;
            ROS_INFO("...recovering PG70 gripper successful.");
        }
        else
        {
            res.success.data = false;
            res.error_message.data = pg70_ctrl_->getErrorMessage();
            ROS_ERROR("...recovering PG70 gripper not successful. error: %s", res.error_message.data.c_str());
        }
    }

    else
    {
        res.success.data = false;
        res.error_message.data = "PG70 gripper not initialized";
        ROS_ERROR("...recovering PG70 gripper not successful. error: %s",res.error_message.data.c_str());
    }

    return true;
}


}
