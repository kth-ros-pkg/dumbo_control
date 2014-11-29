/*
 *  schunk_arm_hw.cpp
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

#include <dumbo_hardware_interface/schunk_arm_hw.h>
#include <urdf/model.h>

namespace dumbo_hardware_interface
{

SchunkArmHW::SchunkArmHW(const ros::NodeHandle &nh,
                                                       boost::shared_ptr<pthread_mutex_t> CAN_mutex,
                                                       boost::shared_ptr<canHandle> CAN_handle) :
    PowerCubeCtrl(CAN_mutex, CAN_handle),
    nh_(nh),
    written_(false),
    module_number_(0)
{
    params_.reset(new PowerCubeCtrlParams());

    getROSParams();
    getRobotDescriptionParams();
}

SchunkArmHW::~SchunkArmHW()
{
}

void SchunkArmHW::getROSParams()
{
    /// get CanBaudrate
    int CanBaudrate;
    if (nh_.hasParam("can_baudrate"))
    {
        nh_.getParam("can_baudrate", CanBaudrate);
    }
    else
    {
        ROS_ERROR("Parameter can_baudrate not set, shutting down node...");
        nh_.shutdown();
    }

    /// get Modul IDs
    XmlRpc::XmlRpcValue ModulIDsXmlRpc;
    std::vector<int> ModulIDs;
    if (nh_.hasParam("module_ids"))
    {
        nh_.getParam("module_ids", ModulIDsXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter module_ids not set, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the ModulIDs
    ModulIDs.resize(ModulIDsXmlRpc.size());
    for (int i = 0; i < ModulIDsXmlRpc.size(); i++)
    {
        ModulIDs[i] = (int)ModulIDsXmlRpc[i];
    }

    /// Initialize parameters
    params_->Init(CanBaudrate, ModulIDs);


    /// Get arm name

    if(nh_.hasParam("arm_name"))
    {
        nh_.getParam("arm_name", arm_name_);
    }

    else
    {
        ROS_ERROR("Parameters arm_name not found, shutting down node");
        nh_.shutdown();
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
        ROS_ERROR("Parameter joint_names not set, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the JointNames
    JointNames.resize(JointNamesXmlRpc.size());
    for (int i = 0; i < JointNamesXmlRpc.size(); i++)
    {
        JointNames[i] = (std::string)JointNamesXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)JointNames.size() != params_->GetDOF())
    {
        ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
        nh_.shutdown();
    }
    params_->SetJointNames(JointNames);


    /// Get max accelerations
    XmlRpc::XmlRpcValue MaxAccelerationsXmlRpc;
    std::vector<double> MaxAccelerations;
    if (nh_.hasParam("max_accelerations"))
    {
        nh_.getParam("max_accelerations", MaxAccelerationsXmlRpc);
    }

    else
    {
        ROS_ERROR("Parameter max_accelerations not set, shutting down node...");
        nh_.shutdown();
    }

    /// Resize and assign of values to the MaxAccelerations
    MaxAccelerations.resize(MaxAccelerationsXmlRpc.size());
    for (int i = 0; i < MaxAccelerationsXmlRpc.size(); i++)
    {
        MaxAccelerations[i] = (double)MaxAccelerationsXmlRpc[i];
    }

    /// Check dimension with with DOF
    if ((int)MaxAccelerations.size() != params_->GetDOF())
    {
        ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
        nh_.shutdown();
    }
    params_->SetMaxAcc(MaxAccelerations);

    /// Get horizon
    double Horizon;
    if (nh_.hasParam("horizon"))
    {
        nh_.getParam("horizon", Horizon);
    }

    else
    {
        /// Horizon in sec
        Horizon = 0.05;
        ROS_WARN("Parameter horizon not available, setting to default value: %f sec", Horizon);
    }
    setHorizon(Horizon);


    // make sure arm select coincides with joint names in the parameter server
    JointNames.clear();
    JointNames = params_->GetJointNames();
    for(int i=0; i<(int)(JointNames.size()); i++)
    {

        if((arm_name_!=(JointNames[i]).substr(0,4))&&(arm_name_!=(JointNames[i]).substr(0,5)))
        {
            ROS_ERROR("arm_name parameter (%s) does not coincide with joint %d name (%s), shutting down node...",
                      arm_name_.c_str(), i, JointNames[i].c_str());
            nh_.shutdown();
        }

    }

    params_->setArmName(arm_name_);

}

void SchunkArmHW::getRobotDescriptionParams()
{
    unsigned int DOF = params_->GetDOF();
    std::vector<std::string> JointNames = params_->GetJointNames();

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

    /// Get max velocities out of urdf model
    std::vector<double> MaxVelocities(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
        MaxVelocities[i] = model.getJoint(JointNames[i].c_str())->limits->velocity;
    }
    ROS_DEBUG("Got max velocities");

    /// Get lower limits out of urdf model
    std::vector<double> LowerLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
        LowerLimits[i] = model.getJoint(JointNames[i].c_str())->limits->lower;
    }
    ROS_DEBUG("Got lower limits");

    // Get upper limits out of urdf model
    std::vector<double> UpperLimits(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
        UpperLimits[i] = model.getJoint(JointNames[i].c_str())->limits->upper;
    }
    ROS_DEBUG("Got upper limits");

    /// Get offsets out of urdf model
    std::vector<double> Offsets(DOF);
    for (unsigned int i = 0; i < DOF; i++)
    {
        Offsets[i] = model.getJoint(JointNames[i].c_str())->calibration->rising.get()[0];
    }
    ROS_DEBUG("Got offsets");


    /// Set parameters
    params_->SetMaxVel(MaxVelocities);
    params_->SetLowerLimits(LowerLimits);
    params_->SetUpperLimits(UpperLimits);
    params_->SetOffsets(Offsets);

}



void SchunkArmHW::registerHandles(hardware_interface::JointStateInterface &js_interface,
                                                 hardware_interface::VelocityJointInterface &vj_interface)
{
    // initialize joint state and commands
    unsigned int dof = params_->GetDOF();
    joint_names_ = params_->GetJointNames();
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
                                        js_interface.getHandle(joint_names_[i]),&joint_velocity_command_[i]));

    }


}

bool SchunkArmHW::connect()
{
    bool ret = init();

    if(ret)
    {
        unsigned int dof = params_->GetDOF();

        // reset joint variables
        for(unsigned int i=0; i<dof; i++)
        {
            joint_positions_[i] = getPositions()[i];
        }

        joint_velocities_ = std::vector<double>(dof, 0.0);
        joint_efforts_ = std::vector<double>(dof, 0.0);

        joint_velocity_command_ = std::vector<double>(dof, 0.0);

        written_ = false;
        module_number_ = 0;
    }
    return ret;
}


bool SchunkArmHW::disconnect()
{
    return close();
}

void SchunkArmHW::read(bool wait_for_response)
{
    // read the joint positions and velocities and store them in the buffer
    if(isInitialized())
    {
        // we need to read the status response msg from the CAN bus
        if(written_)
        {
            bool ret = readState(module_number_, wait_for_response);
            written_ = false;

            if(!ret)
            {
                ROS_ERROR("Error reading status message module %d on %s arm", module_number_,
                          params_->getArmName().c_str());
                return;
            }
        }

        joint_positions_[module_number_] = getPositions()[module_number_];
        joint_velocities_[module_number_] = getVelocities()[module_number_];
    }

}

void SchunkArmHW::write()
{

    if(isInitialized())
    {
        bool ret = moveVel(joint_velocity_command_[module_number_], module_number_, false);

        if(!ret)
        {
            ROS_ERROR("Error writing to module %d on %s arm", module_number_,
                      params_->getArmName().c_str());
            return;
        }

        written_ = true;

        // sequential ordering of commands
        module_number_++;
        if(module_number_ >= params_->GetDOF()) module_number_ = 0;
    }
}

void SchunkArmHW::writeAndRead()
{
    if(isInitialized())
    {

        // clear previously written message
        if(written_)
        {
            read(true);
        }

        bool ret = moveVel(joint_velocity_command_[module_number_], module_number_, true);

        if(!ret)
        {
            ROS_ERROR("Error read/writing to module %d on %s arm", module_number_,
                      params_->getArmName().c_str());
            return;
        }

        // sequential ordering of commands
        module_number_++;
        if(module_number_ >= params_->GetDOF()) module_number_ = 0;

    }

}

}
