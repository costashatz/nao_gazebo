/**
Copyright (c) 2015, Konstantinos Chatzilygeroudis
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
#include <gazebo/sensors/sensors.hh>

// ros_control
#include <control_toolbox/pid.h>

// NAOqi includes
#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/sim_launcher.h>

// General Includes
#include <vector>

namespace gazebo
{
  class GazeboNaoqiControlPlugin : public ModelPlugin
  {
    public:
      GazeboNaoqiControlPlugin();
      ~GazeboNaoqiControlPlugin();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Update();

    private:
      void readSim();
      void writeSim(ros::Time time, ros::Duration period);

      void initSensors();

      void onCameraUpdate(const Sim::CameraSensor* _camera, const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format);

      void onImuUpdate(sensors::ImuSensorPtr _sensor);

      void onSonarUpdate(sensors::RaySensorPtr _gazebo_sonar, const Sim::SonarSensor* _sonar);

      void onFSRUpdate(sensors::ContactSensorPtr _gazebo_fsr, const Sim::FSRSensor* _fsr);

    protected:
      // NAOqi Parameters
      Sim::SimLauncher* naoqi_sim_launcher_;
      Sim::Model* naoqi_model_;
      Sim::HALInterface* naoqi_hal_;

      // NAOqi Joint Actuators/Sensors
      std::vector<const Sim::Joint*> joints_;

      std::vector<const Sim::AngleSensor*> angle_sensors_;

      std::vector<const Sim::AngleActuator*> angle_actuators_;

      const Sim::CameraSensor* top_camera_;
      const Sim::CameraSensor* bottom_camera_;
      const Sim::InertialSensor* inertial_sensor_;
      const Sim::SonarSensor* left_sonar_;
      const Sim::SonarSensor* right_sonar_;
      const Sim::FSRSensor* LFoot_front_left_;
      const Sim::FSRSensor* LFoot_front_right_;
      const Sim::FSRSensor* LFoot_rear_left_;
      const Sim::FSRSensor* LFoot_rear_right_;
      const Sim::FSRSensor* RFoot_front_left_;
      const Sim::FSRSensor* RFoot_front_right_;
      const Sim::FSRSensor* RFoot_rear_left_;
      const Sim::FSRSensor* RFoot_rear_right_;

      // ROS Parameters
      std::vector<control_toolbox::Pid> pid_controllers_;

      // Gazebo joints/sensors
      std::vector<physics::JointPtr> gazebo_joints_;
      sensors::CameraSensorPtr gazebo_top_camera_, gazebo_bottom_camera_;
      sensors::ImuSensorPtr gazebo_imu_;
      sensors::ContactSensorPtr gazebo_lfoot_front_left_, gazebo_lfoot_front_right_, gazebo_lfoot_rear_left_, gazebo_lfoot_rear_right_;
      sensors::ContactSensorPtr gazebo_rfoot_front_left_, gazebo_rfoot_front_right_, gazebo_rfoot_rear_left_, gazebo_rfoot_rear_right_;
      sensors::RaySensorPtr gazebo_left_sonar_, gazebo_right_sonar_;

      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the world
      physics::WorldPtr world_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      event::ConnectionPtr new_top_camera_frame_connection_, new_bottom_camera_frame_connection_;
      event::ConnectionPtr imu_update_connection_;
      event::ConnectionPtr left_sonar_update_connection_, right_sonar_update_connection_;
      event::ConnectionPtr fsr_lfl_update_connection_, fsr_lfr_update_connection_, fsr_lrl_update_connection_, fsr_lrr_update_connection_;
      event::ConnectionPtr fsr_rfl_update_connection_, fsr_rfr_update_connection_, fsr_rrl_update_connection_, fsr_rrr_update_connection_;

      // Gazebo/ROS Parameters
      ros::Duration control_period_;
      ros::Time last_update_sim_time_ros_, last_write_sim_time_ros_;
      math::Vector3 gravity_;

      std::vector<std::string> joints_names_;

      std::string naoqi_path_, naoqi_sim_path_, naoqi_model_type_;
      std::string robot_namespace_;
      int naoqi_port_;

  };
}

#endif
