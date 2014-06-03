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

#include <gazebo_naoqi_control/gazebo_naoqi_control_plugin.h>

using namespace gazebo;

GazeboNaoqiControlPlugin::GazeboNaoqiControlPlugin()
{
  naoqi_sim_launcher_ = new Sim::NAOqiLauncher();

  naoqi_path_ = NAOQI_SDK;
  naoqi_sim_path_ = NAOQI_SIM_SDK;
}

GazeboNaoqiControlPlugin::~GazeboNaoqiControlPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  if(naoqi_sim_launcher_)
    delete naoqi_sim_launcher_;
  if(naoqi_model_)
    delete naoqi_model_;
  if(naoqi_hal_)
    delete naoqi_hal_;
}

void GazeboNaoqiControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  model_ = _parent;

  // Error message if the model couldn't be found
  if (!model_)
  {
    ROS_ERROR("Parent model is NULL! GazeboNaoqiControlPlugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  // Get Gazebo World pointer
  world_ = model_->GetWorld();

  // Get the Gazebo simulation period
  ros::Duration gazebo_period(world_->GetPhysicsEngine()->GetMaxStepSize());

  // Check for controlPeriod element
  if(_sdf->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(_sdf->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if(control_period_ < gazebo_period)
    {
      ROS_ERROR("Desired controller update period is faster than the gazebo simulation period.");
      control_period_ = gazebo_period;
    }
    else if(control_period_ > gazebo_period)
    {
      ROS_WARN("Desired controller update period is slower than the gazebo simulation period.");
    }
  }
  else
  {
    control_period_ = gazebo_period;
  }

  // Check for port element
  if(_sdf->HasElement("port"))
  {
    naoqi_port_ = _sdf->GetElement("port")->Get<int>();
  }
  else
  {
    naoqi_port_ = 9559;
    ROS_WARN("No port element present. Using default: [%d]", naoqi_port_);
  }

  // Check for modelType element
  naoqi_model_type_ = "NAO_H25_V40";
  if(_sdf->HasElement("modelType"))
  {
    naoqi_model_type_ = _sdf->GetElement("modelType")->Get<string>();
  }

  // Load NAOqi model and start simulated NAOqi
  try
  {
    naoqi_model_ = new Sim::Model(naoqi_sim_path_+"/share/alrobotmodel/models/"+naoqi_model_type_+".xml");
    naoqi_hal_ = new Sim::HALInterface(naoqi_model_, naoqi_port_);
    if(!naoqi_sim_launcher_->launch(naoqi_model_, naoqi_port_, naoqi_path_))
    {
      ROS_ERROR("Failed to Launch HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.");
      return;
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Exception while launching HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.\n\t%s", e.what());
    return;
  }

  // Get actuators/sensors from NAOqi
  joints_ = naoqi_model_->joints();
  for(unsigned int i=0;i<joints_.size();i++)
  {
    string name = joints_[i]->name();
    physics::JointPtr joint = model_->GetJoint(name);
    if(joint)
    {
      gazebo_joints_.push_back(joint);
      joints_names_.push_back(name);
    }
  }

  angle_sensors_ = naoqi_model_->angleSensors();
  torque_sensors_ = naoqi_model_->torqueSensors();
  angle_speed_sensors_ = naoqi_model_->angleSpeedSensors();

  angle_actuators_ = naoqi_model_->angleActuators();
  torque_actuators_ = naoqi_model_->torqueActuators();
  angle_speed_actuators_ = naoqi_model_->angleSpeedActuators();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboNaoqiControlPlugin::Update, this));

  ROS_INFO("GazeboNaoqiControlPlugin loaded successfully!");
}

void GazeboNaoqiControlPlugin::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = world_->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    //robot_hw_sim_->readSim(sim_time_ros, sim_period);
    readSim(sim_time_ros, sim_period);
    // Compute the controller commands
    
  }

  // Update the gazebo model with the result of the controller
  // computation
  //robot_hw_sim_->writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

void GazeboNaoqiControlPlugin::readSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    if(gazebo_joints_[i]->GetAngle(0).Radian()!=gazebo_joints_[i]->GetAngle(0).Radian())
      continue;
    naoqi_hal_->sendAngleSensorValue(naoqi_model_->angleSensor(joints_names_[i]), gazebo_joints_[i]->GetAngle(0).Radian());
  }
}

void GazeboNaoqiControlPlugin::writeSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    double angle = naoqi_hal_->fetchAngleActuatorValue(naoqi_model_->angleActuator(joints_names_[i]));
    if(angle!=angle)
    {
      angle = naoqi_model_->angleActuator(joints_names_[i])->startValue();
    }
    gazebo_joints_[i]->SetAngle(0,math::Angle(angle));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboNaoqiControlPlugin);