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

// Includes
#include <gazebo_naoqi_control/gazebo_naoqi_control_plugin.h>
#include <angles/angles.h>
#include <alerror/alerror.h>
#include <gazebo/rendering/Camera.hh>
#include <boost/filesystem.hpp>

namespace gazebo
{

GazeboNaoqiControlPlugin::GazeboNaoqiControlPlugin()
{
  // Create NAOqi simulation launcher
  naoqi_sim_launcher_ = new Sim::SimLauncher();

  // Get paths from CMake
  naoqi_path_ = NAOQI_SDK;
  naoqi_sim_path_ = NAOQI_SIM_SDK;
}

GazeboNaoqiControlPlugin::~GazeboNaoqiControlPlugin()
{
  // Disconnect from Gazebo Update Loop
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  // Disconnect from Sensors Update Loops
  if(gazebo_top_camera_)
    gazebo_top_camera_->GetCamera()->DisconnectNewImageFrame(new_top_camera_frame_connection_);
  if(gazebo_bottom_camera_)
    gazebo_bottom_camera_->GetCamera()->DisconnectNewImageFrame(new_bottom_camera_frame_connection_);

  if(gazebo_imu_)
    gazebo_imu_->DisconnectUpdated(imu_update_connection_);

  if(gazebo_left_sonar_)
    gazebo_left_sonar_->GetLaserShape()->DisconnectNewLaserScans(left_sonar_update_connection_);
  if(gazebo_right_sonar_)
    gazebo_right_sonar_->GetLaserShape()->DisconnectNewLaserScans(right_sonar_update_connection_);

  if(gazebo_lfoot_front_left_)
    gazebo_lfoot_front_left_->DisconnectUpdated(fsr_lfl_update_connection_);
  if(gazebo_lfoot_front_right_)
    gazebo_lfoot_front_right_->DisconnectUpdated(fsr_lfr_update_connection_);
  if(gazebo_lfoot_rear_left_)
    gazebo_lfoot_rear_left_->DisconnectUpdated(fsr_lrl_update_connection_);
  if(gazebo_lfoot_rear_right_)
    gazebo_lfoot_rear_right_->DisconnectUpdated(fsr_lrr_update_connection_);

  if(gazebo_rfoot_front_left_)
    gazebo_rfoot_front_left_->DisconnectUpdated(fsr_rfl_update_connection_);
  if(gazebo_rfoot_front_right_)
    gazebo_rfoot_front_right_->DisconnectUpdated(fsr_rfr_update_connection_);
  if(gazebo_rfoot_rear_left_)
    gazebo_rfoot_rear_left_->DisconnectUpdated(fsr_rrl_update_connection_);
  if(gazebo_rfoot_rear_right_)
    gazebo_rfoot_rear_right_->DisconnectUpdated(fsr_rrr_update_connection_);

  // Delete Simulation Objects if necessary
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

  // Get Gravity vector
  gravity_ = world_->GetPhysicsEngine()->GetGravity();

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
  	boost::filesystem::path naoqi_sim_path = naoqi_sim_path_;
  	naoqi_sim_path /= "share/alrobotmodel/models/"+naoqi_model_type_+".xml";
    // Get Model from Simulation SDK dir
    naoqi_model_ = new Sim::Model(naoqi_sim_path.string());
    // Create HAL interface
    naoqi_hal_ = new Sim::HALInterface(naoqi_model_, naoqi_port_);
    // Launch Simulation
    if(!naoqi_sim_launcher_->launch(naoqi_model_, naoqi_port_, naoqi_sim_path_))
    {
      ROS_ERROR("Failed to Launch HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.");
      return;
    }
  }
  catch(const AL::ALError& e)
  {
    ROS_ERROR("Exception while launching HAL or NAOqi. GazeboNaoqiControlPlugin could not be loaded.\n\t%s", e.what());
    return;
  }

  // Get actuators/sensors from NAOqi
  angle_sensors_ = naoqi_model_->angleSensors();

  angle_actuators_ = naoqi_model_->angleActuators();

  joints_ = naoqi_model_->joints();
  // Create Gazebo Joints
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

  // Initialize actuators
  for(std::vector<const Sim::AngleActuator*>::const_iterator it =
      angle_actuators_.begin(); it != angle_actuators_.end(); ++it)
  {
    float actuatorPosition = naoqi_hal_->fetchAngleActuatorValue(*it);
    if(actuatorPosition != actuatorPosition)
    {
      actuatorPosition = (*it)->startValue();
    }
    const Sim::AngleSensor* sensor = naoqi_model_->angleSensor((*it)->name());
    naoqi_hal_->sendAngleSensorValue(sensor, actuatorPosition);
  }

  // Initialize Sensors
  initSensors();


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboNaoqiControlPlugin::Update, this));

  ROS_INFO("GazeboNaoqiControlPlugin loaded successfully!");
}

void GazeboNaoqiControlPlugin::initSensors()
{
  // Activate Camera Sensors
  std::vector<const Sim::CameraSensor*> camera_sensors = naoqi_model_->cameraSensors();
  for(int i=0;i<camera_sensors.size();i++)
  {
    sensors::CameraSensorPtr cam = (boost::dynamic_pointer_cast<sensors::CameraSensor>(sensors::SensorManager::Instance()->GetSensor(camera_sensors[i]->name())));

    if(cam)
    {
      // Subscribe to camera updates
      if(camera_sensors[i]->name() == "CameraTop")
      {
        top_camera_ = camera_sensors[i];
        gazebo_top_camera_ = cam;
        new_top_camera_frame_connection_ = cam->GetCamera()->ConnectNewImageFrame(boost::bind(&GazeboNaoqiControlPlugin::onCameraUpdate, this, top_camera_, _1, _2, _3, _4, _5));
      }
      else if(camera_sensors[i]->name() == "CameraBottom")
      {
        bottom_camera_ = camera_sensors[i];
        gazebo_bottom_camera_ = cam;
        new_bottom_camera_frame_connection_ = cam->GetCamera()->ConnectNewImageFrame(boost::bind(&GazeboNaoqiControlPlugin::onCameraUpdate, this, bottom_camera_, _1, _2, _3, _4, _5));
      }
      else
        continue;
      cam->SetActive(true);
    }
  }

  // Activate IMU - I assume only 1 IMU (TO-DO: Check cases where more exist)
  std::vector<const Sim::InertialSensor*> inertial_sensors = naoqi_model_->inertialSensors();
  if(inertial_sensors.size()>=1)
  {
    sensors::ImuSensorPtr imu = (boost::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor("imu")));

    if(imu)
    {
      imu_update_connection_ = imu->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onImuUpdate, this, imu));
      gazebo_imu_ = imu;
      inertial_sensor_ = inertial_sensors[0];
      imu->SetActive(true);
    }
  }

  // Activate FSRs
  std::vector<const Sim::FSRSensor*> fsr_sensors = naoqi_model_->fsrSensors();
  for(int i=0;i<fsr_sensors.size();i++)
  {
    sensors::ContactSensorPtr fsr = (boost::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensor(fsr_sensors[i]->name())));

    if(fsr)
    {
      std::string name = fsr_sensors[i]->name();
      if(name == "RFoot/FSR/RearLeft")
      {
        gazebo_rfoot_rear_left_ = fsr;
        RFoot_rear_left_ = fsr_sensors[i];
        fsr_rrl_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_rfoot_rear_left_, RFoot_rear_left_));
      }
      else if(name == "RFoot/FSR/RearRight")
      {
        gazebo_rfoot_rear_right_ = fsr;
        RFoot_rear_right_ = fsr_sensors[i];
        fsr_rrr_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_rfoot_rear_right_, RFoot_rear_right_));
      }
      else if(name == "RFoot/FSR/FrontLeft")
      {
        gazebo_rfoot_front_left_ = fsr;
        RFoot_front_left_ = fsr_sensors[i];
        fsr_rfl_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_rfoot_front_left_, RFoot_front_left_));
      }
      else if(name == "RFoot/FSR/FrontRight")
      {
        gazebo_rfoot_front_right_ = fsr;
        RFoot_front_right_ = fsr_sensors[i];
        fsr_rfr_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_rfoot_front_right_, RFoot_front_right_));
      }
      else if(name == "LFoot/FSR/RearLeft")
      {
        gazebo_lfoot_rear_left_ = fsr;
        LFoot_rear_left_ = fsr_sensors[i];
        fsr_lrl_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_lfoot_rear_left_, LFoot_rear_left_));
      }
      else if(name == "LFoot/FSR/RearRight")
      {
        gazebo_lfoot_rear_right_ = fsr;
        LFoot_rear_right_ = fsr_sensors[i];
        fsr_lrr_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_lfoot_rear_right_, LFoot_rear_right_));
      }
      else if(name == "LFoot/FSR/FrontLeft")
      {
        gazebo_lfoot_front_left_ = fsr;
        LFoot_front_left_ = fsr_sensors[i];
        fsr_lfl_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_lfoot_front_left_, LFoot_front_left_));
      }
      else if(name == "LFoot/FSR/FrontRight")
      {
        gazebo_lfoot_front_right_ = fsr;
        LFoot_front_right_ = fsr_sensors[i];
        fsr_lfr_update_connection_ = fsr->ConnectUpdated(boost::bind(&GazeboNaoqiControlPlugin::onFSRUpdate, this, gazebo_lfoot_front_right_, LFoot_front_right_));
      }
      fsr->SetActive(true);
    }
  }

  // Activate Sonars
  std::vector<const Sim::SonarSensor*> sonar_sensors = naoqi_model_->sonarSensors();
  for(int i=0;i<sonar_sensors.size();i++)
  {
    sensors::RaySensorPtr sonar = (boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(sonar_sensors[i]->name())));
    if(sonar)
    {
      if(sonar_sensors[i]->name() == "Sonar/Left")
      {
        left_sonar_ = sonar_sensors[i];
        gazebo_left_sonar_ = sonar;
        left_sonar_update_connection_ = sonar->GetLaserShape()->ConnectNewLaserScans(boost::bind(&GazeboNaoqiControlPlugin::onSonarUpdate, this, gazebo_left_sonar_, left_sonar_));
      }
      else if(sonar_sensors[i]->name() == "Sonar/Right")
      {
        right_sonar_ = sonar_sensors[i];
        gazebo_right_sonar_ = sonar;
        right_sonar_update_connection_ = sonar->GetLaserShape()->ConnectNewLaserScans(boost::bind(&GazeboNaoqiControlPlugin::onSonarUpdate, this, gazebo_right_sonar_, right_sonar_));
      }
      sonar->SetActive(true);
    }
  }
}

void GazeboNaoqiControlPlugin::onCameraUpdate(const Sim::CameraSensor* _camera, const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format)
{
  int res;
  if(_width==80)
    res = Sim::RES_80_60;
  else if(_width==160)
    res = Sim::RES_160_120;
  else if(_width==320)
    res = Sim::RES_320_240;
  else if(_width==640)
    res = Sim::RES_640_480;
  else if(_width==1280)
    res = Sim::RES_1280_960;
  else
    res = Sim::RES_UNKNOWN;

  // TO-DO: Update Gazebo camera to match NAOqi's resolution
  int r = naoqi_hal_->cameraResolution(_camera);
  if(r!=res)
  {
    ROS_WARN("Mismatch in dimensions when sending image to camera '%s': %d vs %d", _camera->name().c_str(), res, r);
    return;
  }
  // Send image to NAOqi
  naoqi_hal_->sendCameraSensorValue(_camera, _image, (Sim::CameraResolution)res, Sim::COL_SPACE_RGB);
}

void GazeboNaoqiControlPlugin::onImuUpdate(sensors::ImuSensorPtr _sensor)
{
  //Update IMU
  // Get IMU data from Gazebo
  math::Vector3 gyro = gazebo_imu_->GetAngularVelocity();
  math::Vector3 acc = gazebo_imu_->GetLinearAcceleration();
  math::Quaternion orient = gazebo_imu_->GetOrientation();

  std::vector<float> vals;
  // AngleX, AngleY, [AngleZ - not in V40], AccX, AccY, AccZ, GyroX, GyroY, [GyroZ - not in V40]

  vals.push_back(orient.GetRoll());
  vals.push_back(orient.GetPitch());
  // if(naoqi_model_type_ == "NAOH25V50") //Should be working for V50 - it doesn't
  //   vals.push_back(orient.GetYaw());
  vals.push_back(acc[0]);
  vals.push_back(acc[1]);
  vals.push_back(-acc[2]);
  vals.push_back(gyro[0]);
  vals.push_back(gyro[1]);
  // if(naoqi_model_type_ == "NAOH25V50") //Should be working for V50 - it doesn't
  //   vals.push_back(gyro[2]);

  // Send IMU data to NAOqi
  naoqi_hal_->sendInertialSensorValues(inertial_sensor_, vals);
}

void GazeboNaoqiControlPlugin::onSonarUpdate(sensors::RaySensorPtr _gazebo_sonar, const Sim::SonarSensor* _sonar)
{
  // Update Left Sonar
  // Get Sonar range from Gazebo
  double val = 0.0;
  int N = _gazebo_sonar->GetRayCount();
  for(int i=0;i<N;i++)
  {
    val += _gazebo_sonar->GetRange(i);
  }
  val /= double(N);

  // Send Sonar range to NAOqi
  naoqi_hal_->sendSonarSensorValue(_sonar, float(val));
}

void GazeboNaoqiControlPlugin::onFSRUpdate(sensors::ContactSensorPtr _gazebo_fsr, const Sim::FSRSensor* _fsr)
{
  // Update FSR
  // Get FSR data from Gazebo
  msgs::Contacts contacts = _gazebo_fsr->GetContacts();
  math::Vector3 force = math::Vector3::Zero;
  for(int j=0;j<contacts.contact_size();j++)
  {
    int k=0;
    for(int k=0;k<contacts.contact(j).position_size();k++)
    {
      // TO-DO: Needs checking
      msgs::Vector3d v = contacts.contact(j).wrench(k).body_1_wrench().force();
      force += math::Vector3(v.x(), v.y(), v.z());
      // v = contacts.contact(j).wrench(k).body_2_wrench().force();
      // force += math::Vector3(v.x(), v.y(), v.z());
    }
  }
  double g = gravity_[2];
  if(g<0)
    g = -g;
  else if(g==0.0)
    g = 1.0;
  // Send FSR data to NAOqi
  naoqi_hal_->sendFSRSensorValue(_fsr, force[2]/g);
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
    readSim();
  }

  // Update the gazebo model with commands from NAOqi
  writeSim(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
}

void GazeboNaoqiControlPlugin::readSim()
{
  // Read joint states from Gazebo and send them to NAOqi
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    double angle = gazebo_joints_[i]->GetAngle(0).Radian();
    if(angle!=angle)
      continue;
    if(naoqi_model_->angleSensor(joints_names_[i])) //sometimes we cannot get the handle
      naoqi_hal_->sendAngleSensorValue(naoqi_model_->angleSensor(joints_names_[i]), angle);
  }
}

void GazeboNaoqiControlPlugin::writeSim(ros::Time time, ros::Duration period)
{
  // Get actuator commands from NAOqi and write them in Gazebo
  // TO-DO: Implement LED, Sonar and other commads
  for(unsigned int i=0;i<gazebo_joints_.size();i++)
  {
    if(!naoqi_model_->angleActuator(joints_names_[i])) //sometimes we cannot get the handle
      continue;

    double angle = naoqi_hal_->fetchAngleActuatorValue(naoqi_model_->angleActuator(joints_names_[i]));
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

}
