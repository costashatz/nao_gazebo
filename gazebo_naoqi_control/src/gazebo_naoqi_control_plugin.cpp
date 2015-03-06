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

using namespace gazebo;

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
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

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
    // Get Model from Simulation SDK dir
    naoqi_model_ = new Sim::Model(naoqi_sim_path_+"share/alrobotmodel/models/"+naoqi_model_type_+".xml");
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
  camera_sensors_ = naoqi_model_->cameraSensors();
  for(int i=0;i<camera_sensors_.size();i++)
  {
    sensors::CameraSensorPtr cam = (boost::dynamic_pointer_cast<sensors::CameraSensor>(sensors::SensorManager::Instance()->GetSensor(camera_sensors_[i]->name())));

    if(cam)
    {
      cam->SetActive(true);
      gazebo_cameras_.push_back(cam);
    }
  }

  // Activate IMU - I assume only 1 IMU (TO-DO: Check cases where more exist)
  inertial_sensors_ = naoqi_model_->inertialSensors();
  if(inertial_sensors_.size()>=1)
  {
    sensors::ImuSensorPtr imu = (boost::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor("imu")));

    if(imu)
    {
      imu->SetActive(true);
      gazebo_imu_.push_back(imu);
    }
  }

  // Activate FSRs
  fsr_sensors_ = naoqi_model_->fsrSensors();
  for(int i=0;i<fsr_sensors_.size();i++)
  {
    sensors::ContactSensorPtr fsr = (boost::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensor(fsr_sensors_[i]->name())));

    if(fsr)
    {
      fsr->SetActive(true);
      gazebo_fsrs_.push_back(fsr);
    }
  }

  // Activate Sonars
  sonar_sensors_ = naoqi_model_->sonarSensors();
  for(int i=0;i<sonar_sensors_.size();i++)
  {
    sensors::RaySensorPtr sonar = (boost::dynamic_pointer_cast<sensors::RaySensor>(sensors::SensorManager::Instance()->GetSensor(sonar_sensors_[i]->name())));
    if(sonar)
    {
      sonar->SetActive(true);
      gazebo_sonars_.push_back(sonar);
    }
  }
}

void GazeboNaoqiControlPlugin::updateSensors()
{
  // Update Cameras
  for(int i=0;i<gazebo_cameras_.size();i++)
  {
    int width = gazebo_cameras_[i]->GetImageWidth();
    int height = gazebo_cameras_[i]->GetImageHeight();
    // Get Image from Gazebo
    unsigned char* tmp = (unsigned char*)gazebo_cameras_[i]->GetImageData();
    if(!tmp)
    {
      ROS_WARN("NULL image returned from Gazebo camera '%s'", camera_sensors_[i]->name().c_str());
      continue;
    }
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

    // TO-DO: Update Gazebo camera to match NAOqi's resolution
    int r = naoqi_hal_->cameraResolution(camera_sensors_[i]);
    if(r!=res)
    {
      ROS_WARN("Mismatch in dimensions when sending image to camera '%s': %d vs %d", camera_sensors_[i]->name().c_str(), res, r);
      continue;
    }
    // Send image to NAOqi
    naoqi_hal_->sendCameraSensorValue(camera_sensors_[i], tmp, (Sim::CameraResolution)res, Sim::COL_SPACE_RGB);
  }

  //Update IMU
  if(gazebo_imu_.size()>=1)
  {
    // Get IMU data from Gazebo
    math::Vector3 gyro = gazebo_imu_[0]->GetAngularVelocity();
    math::Vector3 acc = gazebo_imu_[0]->GetLinearAcceleration();
    math::Quaternion orient = gazebo_imu_[0]->GetOrientation();

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
    naoqi_hal_->sendInertialSensorValues(inertial_sensors_[0], vals);
  }

  math::Vector3 gravity = world_->GetPhysicsEngine()->GetGravity();

  // Update FSRs
  for(int i=0;i<gazebo_fsrs_.size();i++)
  {
    // Get FSR data from Gazebo
    msgs::Contacts contacts = gazebo_fsrs_[i]->GetContacts();
    math::Vector3 force = math::Vector3::Zero;
    for(int j=0;j<contacts.contact_size();j++)
    {
      int k=0;
      for(int k=0;k<contacts.contact(j).position_size();k++)
      {
        // TO-DO: Needs checking
        msgs::Vector3d v = contacts.contact(j).wrench(k).body_1_wrench().force();
        force += math::Vector3(v.x(), v.y(), v.z());
        v = contacts.contact(j).wrench(k).body_2_wrench().force();
        force += math::Vector3(v.x(), v.y(), v.z());
      }
    }
    double g = gravity[2];
    if(g<0)
      g = -g;
    else if(g==0.0)
      g = 1.0;
    // Send FSR data to NAOqi
    naoqi_hal_->sendFSRSensorValue(fsr_sensors_[i], force.GetLength()/g);
  }

  // Update Sonars
  for(int i=0;i<gazebo_sonars_.size();i++)
  {
    // Get Sonar range from Gazebo
    std::vector<double> tmp;
    gazebo_sonars_[i]->GetRanges(tmp);

    double val = 0.0;
    for(int j=0;j<tmp.size();j++)
      val += tmp[j];
    val /= tmp.size();
    // Send Sonar range to NAOqi
    naoqi_hal_->sendSonarSensorValue(sonar_sensors_[i], float(val));
  }
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
    boost::thread t1(&GazeboNaoqiControlPlugin::readSim, this);
    // Update sensors in different thread
    boost::thread t2(&GazeboNaoqiControlPlugin::updateSensors, this);

    t1.join();
    t2.join();
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