/**
Copyright (c) 2014, Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer 
    in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived 
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef GAZEBO_NAOQI_CONTROL_GAZEBO_NAOQI_CONTROL_PLUGIN
#define GAZEBO_NAOQI_CONTROL_GAZEBO_NAOQI_CONTROL_PLUGIN

// ROS includes
#include <ros/ros.h>
#include <ros/duration.h>
#include <pluginlib/class_loader.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ros_control
#include <gazebo_ros_control/robot_hw_sim.h>
#include <transmission_interface/transmission_parser.h>

// NAOqi includes
#include <qi/os.hpp>
#include <alnaosim/alnaosim.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/naoqi_launcher.h>

#include <vector>

using std::vector;
using std::string;

namespace gazebo
{
  class GazeboNaoqiControlPlugin : public ModelPlugin
  {
    public:
      GazeboNaoqiControlPlugin();
      ~GazeboNaoqiControlPlugin();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Update();

      void readSim(ros::Time time, ros::Duration period);
      void writeSim(ros::Time time, ros::Duration period);

    private:
      // Parameters
      Sim::NAOqiLauncher* naoqi_sim_launcher_;
      Sim::Model* naoqi_model_;
      Sim::HALInterface* naoqi_hal_;
      string naoqi_path_, naoqi_sim_path_, naoqi_model_type_;
      int naoqi_port_;
      ros::Duration control_period_;
      ros::Time last_update_sim_time_ros_, last_write_sim_time_ros_;

      vector<string> joints_names_;
      vector<const Sim::Joint*> joints_;

      vector<const Sim::AngleSensor*> angle_sensors_;
      vector<const Sim::TorqueSensor*> torque_sensors_;
      vector<const Sim::AngleSpeedSensor*> angle_speed_sensors_;

      vector<const Sim::AngleActuator*> angle_actuators_;
      vector<const Sim::TorqueActuator*> torque_actuators_;
      vector<const Sim::AngleSpeedActuator*> angle_speed_actuators_;

      vector<physics::JointPtr> gazebo_joints_;

      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the world
      physics::WorldPtr world_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

  };
}

#endif