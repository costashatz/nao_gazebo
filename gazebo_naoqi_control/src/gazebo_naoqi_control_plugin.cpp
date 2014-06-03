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

  naoqi_path_ = "/home/costas/Workspaces/nao/devtools/naoqi-sdk-1.14.5-linux64";
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

  // Check for robotNamespace element
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<string>();
  }
  else
  {
    robot_namespace_ = model_->GetName(); // default
  }

  // Check for robotParam element
  if (_sdf->HasElement("robotParam"))
  {
    robot_description_ = _sdf->GetElement("robotParam")->Get<string>();
  }
  else
  {
    robot_description_ = "robot_description"; // default
  }

  // // Check for robotSimType element
  // if(_sdf->HasElement("robotSimType"))
  // {
  //   robot_hw_sim_type_str_ = _sdf->Get<string>("robotSimType");
  // }
  // else
  // {
  //   robot_hw_sim_type_str_ = "gazebo_ros_control/DefaultRobotHWSim";
  // }

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

  // Check for ModelType element
  naoqi_model_type_ = "NAO_H25_V40";
  if(_sdf->HasElement("ModelType"))
  {
    naoqi_model_type_ = _sdf->GetElement("ModelType")->Get<string>();
  }

  // // Node handle
  // model_node_handle_ = ros::NodeHandle(robot_namespace_);

  // // Get URDF string
  // const string urdf_string = getURDF(robot_description_);
  // if (!parseTransmissionsFromURDF(urdf_string))
  // {
  //   ROS_ERROR("Error parsing URDF. GazeboNaoqiControlPlugin could not be loaded.");
  //   return;
  // }

  // // Load the RobotHWSim abstraction to interface the controllers with the gazebo model
  // try
  // {
  //   robot_hw_sim_loader_.reset
  //     (new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
  //       ("gazebo_ros_control",
  //         "gazebo_ros_control::RobotHWSim"));

  //   robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);
  //   urdf::Model urdf_model;
  //   const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

  //   if(!robot_hw_sim_->initSim(robot_namespace_, model_node_handle_, model_, urdf_model_ptr, transmissions_))
  //   {
  //     ROS_ERROR("Could not initialize robot simulation interface. GazeboNaoqiControlPlugin could not be loaded.");
  //     return;
  //   }

    // Load NAOqi model and start simulated NAOqi
    try
    {
      naoqi_model_ = new Sim::Model("/home/costas/Workspaces/ros_hydro/catkin_ws/src/nao_gazebo/gazebo_naoqi_control/models/"+naoqi_model_type_+".xml");
      naoqi_hal_ = new Sim::HALInterface(naoqi_model_, naoqi_port_);
      if(!naoqi_sim_launcher_->launch(naoqi_model_, naoqi_port_, naoqi_path_))
      {
        ROS_ERROR("Failed to Launch HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.");
        return;
      }
      ROS_INFO("Teleio");
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Exception while launching HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.\n\t\t%s", e.what());
      return;
    }

    ROS_INFO("JOINT READING");
    // Get actuators/sensors from NAOqi
    joints_ = naoqi_model_->joints();
    for(unsigned int i=0;i<joints_.size();i++)
    {
      string name = joints_[i]->name();
      joints_names_.push_back(name);
      gazebo_joints_.push_back(model_->GetJoint(name));
    }

    ROS_INFO("SENSROS");
    angle_sensors_ = naoqi_model_->angleSensors();
    torque_sensors_ = naoqi_model_->torqueSensors();
    angle_speed_sensors_ = naoqi_model_->angleSpeedSensors();

    ROS_INFO("ACTUATROS");
    angle_actuators_ = naoqi_model_->angleActuators();
    torque_actuators_ = naoqi_model_->torqueActuators();
    angle_speed_actuators_ = naoqi_model_->angleSpeedActuators();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboNaoqiControlPlugin::Update, this));

  // }
  // catch(pluginlib::LibraryLoadException &ex)
  // {
  //   ROS_FATAL_STREAM_NAMED("gazebo_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
  // }

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

// Get the URDF XML from the parameter server
string GazeboNaoqiControlPlugin::getURDF(string param_name) const
{
  string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    string search_param_name;
    if (model_node_handle_.searchParam(param_name, search_param_name))
    {
      model_node_handle_.getParam(search_param_name, urdf_string);
    }
    else
    {
      model_node_handle_.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }

  return urdf_string;
}

// Get Transmissions from the URDF
bool GazeboNaoqiControlPlugin::parseTransmissionsFromURDF(const string& urdf_string)
{
  //transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

void GazeboNaoqiControlPlugin::readSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    naoqi_hal_->sendAngleSensorValue(angle_sensors_[i], gazebo_joints_[i]->GetAngle(0).Radian());
  }
}

void GazeboNaoqiControlPlugin::writeSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    float effort = naoqi_hal_->fetchTorqueActuatorValue(torque_actuators_[i]);
    gazebo_joints_[i]->SetForce(0,effort);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboNaoqiControlPlugin);