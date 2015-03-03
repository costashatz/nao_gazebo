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
#include <angles/angles.h>

using namespace gazebo;

GazeboNaoqiControlPlugin::GazeboNaoqiControlPlugin()
{
  naoqi_sim_launcher_ = new Sim::SimLauncher();

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
  ros::NodeHandle model_nh;
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
  // world_->GetPhysicsEngine()->SetGravity(gazebo::math::Vector3(0,0,0));

  // Check for robot namespace
  robot_namespace_ = "/";
  if(_sdf->HasElement("robotNamespace"))
  {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

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
  naoqi_model_type_ = "NAOH25V40";
  if(_sdf->HasElement("modelType"))
  {
    naoqi_model_type_ = _sdf->GetElement("modelType")->Get<std::string>();
  }

  // Load NAOqi model and start simulated NAOqi
  try
  {
    naoqi_model_ = new Sim::Model(naoqi_sim_path_+"share/alrobotmodel/models/"+naoqi_model_type_+".xml");
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
  angle_sensors_ = naoqi_model_->angleSensors();

  angle_actuators_ = naoqi_model_->angleActuators();

  joints_ = naoqi_model_->joints();
  for(unsigned int i=0;i<angle_actuators_.size();i++)
  {
    std::string name = angle_actuators_[i]->name();
    physics::JointPtr joint = model_->GetJoint(name);
    if(joint)
    {
      gazebo_joints_.push_back(joint);
      joints_names_.push_back(name);
      const ros::NodeHandle nh(model_nh, std::string(robot_namespace_+"/gazebo_ros_control/pid_gains/")+name);
      double p, i,d ;
      // TO-DO: include i_clamp e.t.c.
      nh.param("p", p, 0.0);
      nh.param("i", i, 0.0);
      nh.param("d", d, 0.0);

      pid_controllers_.push_back(control_toolbox::Pid(p, i, d));
    }
  }

  initSensors();


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboNaoqiControlPlugin::Update, this));

  ROS_INFO("GazeboNaoqiControlPlugin loaded successfully!");
}

void GazeboNaoqiControlPlugin::initSensors()
{
  camera_sensors_ = naoqi_model_->cameraSensors();

  inertial_sensors_ = naoqi_model_->inertialSensors();
}

void GazeboNaoqiControlPlugin::updateSensors()
{
  // Update Cameras
  for(int i=0;i<camera_sensors_.size();i++)
  {
    sensors::CameraSensorPtr cam = (boost::dynamic_pointer_cast<sensors::CameraSensor>(sensors::SensorManager::Instance()->GetSensor(camera_sensors_[i]->name())));
    if(cam)
    {
      int width = cam->GetImageWidth();
      int height = cam->GetImageHeight();
      unsigned char* tmp = new unsigned char[width*height];
      tmp = (unsigned char*)cam->GetImageData();
      int res;
      if(width==80)
        res = Sim::RES_80_60;
      else if(width==160)
        res = Sim::RES_160_120;
      else if(width==320)
        res = Sim::RES_320_240;
      else if(width==640)
        res = Sim::RES_640_480;
      else if(width==1280)
        res = Sim::RES_1280_960;
      else
        res = Sim::RES_UNKNOWN;
      // int* buffer = new int[width*height];
      // naoqi_hal_->cameraBufferSize(camera_sensors_[i], buffer, &width, &height);
      int r = naoqi_hal_->cameraResolution(camera_sensors_[i]);
      if(r!=res)
      {
        ROS_ERROR("Mismatch in dimensions: %d vs %d", res, r);
        continue;
      }
      if(!tmp)
      {
        ROS_ERROR("NULL image");
        continue;
      }
      naoqi_hal_->sendCameraSensorValue(camera_sensors_[i], tmp, (Sim::CameraResolution)res, Sim::COL_SPACE_RGB);
    }
  }

  //Update IMU
  if(inertial_sensors_.size()>=1)
  {
    sensors::ImuSensorPtr imu = (boost::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor("imu")));
    math::Vector3 angle = imu->GetAngularVelocity();
    math::Vector3 acc = imu->GetLinearAcceleration();
    math::Quaternion gyro = imu->GetOrientation();

    std::vector<float> vals;
    // AngleX, AngleY, [AngleZ - not in V40], AccX, AccY, AccZ, GyroX, GyroY, [GyroZ - not in V40]

    vals.push_back(angle[0]);
    vals.push_back(angle[1]);
    vals.push_back(acc[0]);
    vals.push_back(acc[1]);
    vals.push_back(acc[2]);
    vals.push_back(gyro.GetPitch());
    vals.push_back(gyro.GetRoll());

    naoqi_hal_->sendInertialSensorValues(inertial_sensors_[0], vals);
  }
}

void GazeboNaoqiControlPlugin::Update()
{
  updateSensors();
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = world_->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // Check if we should update the controllers
  if(sim_period >= control_period_) {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // Update the robot simulation with the state of the gazebo model
    readSim(sim_time_ros, sim_period);
  }

  // Update the gazebo model with commands from NAOqi
  writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

void GazeboNaoqiControlPlugin::readSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    double angle = gazebo_joints_[i]->GetAngle(0).Radian();
    if(angle!=angle)
      continue;
    if(naoqi_model_->angleSensor(joints_names_[i])) //some are passive joints
      naoqi_hal_->sendAngleSensorValue(naoqi_model_->angleSensor(joints_names_[i]), angle);
  }
}

void GazeboNaoqiControlPlugin::writeSim(ros::Time time, ros::Duration period)
{
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    if(!naoqi_model_->angleActuator(joints_names_[i]))
    {
       //some are passive joints
      continue;
    }
    double angle = naoqi_hal_->fetchAngleActuatorValue(naoqi_model_->angleActuator(joints_names_[i]));
    if(joints_names_[i]=="RHipYawPitch")
      angle = naoqi_hal_->fetchAngleActuatorValue(naoqi_model_->angleActuator("LHipYawPitch"));
    if(angle!=angle)
    {
      angle = naoqi_model_->angleActuator(joints_names_[i])->startValue();
    }
    double a = gazebo_joints_[i]->GetAngle(0).Radian();
    if(a!=a)
      a = angle;
    double error = angle-a;
    double effort = gazebo::math::clamp(pid_controllers_[i].computeCommand(error, period), -2.0, 2.0);
    gazebo_joints_[i]->SetForce(0, effort);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboNaoqiControlPlugin);