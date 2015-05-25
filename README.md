#nao_gazebo
 
##ROS Packages to simulate Aldebaran's NAO Humanoid in Gazebo through NAOqi Simulator SDK
 
  - Supports **all versions and body types of NAO**.
  - **Written purely in C++**.
  - **Standalone ROS package**


Version
----

0.0.1 (Beta)

Requirements
-----------

nao_gazebo requires several packages to be installed in order to work properly:

* [ROS] - ROS >= **Hydro**
* [ROS Control] - **Version >=0.6.0** [control_toolbox should be installed]
* [Gazebo] - **Version 2.2.5** - *Should work on newer versions but haven't tested it*
* [NAO meshes] - For visualization purposes

Building/Compiling
------------------

You need to have either environmental variable **AL_SIM_DIR** or CMake option **NAOqi_SIM_DIR** pointing to the C++ NAOqi Simulator SDK path.

Also, you need to have either environmental variable **AL_DIR** or CMake option **NAOqi_DIR** pointing to the C++ NAOqi SDK path.

Check *cmake/FindNAOqiSIM.cmake* for more details.

Basic Usage
--------------

###Bringup nao_gazebo
```sh
roslaunch gazebo_naoqi_control nao_gazebo.launch
```

This will open up Gazebo with NAO model loaded and simulation paused! You need to open Choregraphe (or any other driver) to give commands to NAO as you would give to a real robot.

**BE CAREFUL:**

NAO at simulation start up is stiffened off. You need to connect to Choreographe and enable stiffness (wake up the robot), before the robot falls down. DO NOT enable stiffness with the simulation paused. It will cause NAOqi to be stalled and no further commands can be issued!

###Visualize modified model/urdf
```sh
roslaunch gazebo_naoqi_control display.launch
```

This will open up RViz with NAO model loaded for debugging purposes.

URDF Modifications
------------------
Please feel free to check *models/nao.xacro* file for modifying for example sensor properties...

###Plugin SDF elements
* **robotNamespace** - provides the robot namespace
* **controlPeriod** - control period for reading/writing sensor/actuator values
* **port** - port to initialize NAOqi
* **modelType** - type of model to load. Check the path in simulator SDK: *${SIMULATOR_SDK}/share/alrobotmodel/models/*

###Load plugin in URDF

Classic gazebo model plugin insertion.. See example in **models/model.xacro**. In practice create a file like **model.xacro** and tune the xacro properties.

```
  <gazebo>
    <plugin filename="libgazebo_naoqi_control.so" name="gazebo_naoqi_control">
      <robotNamespace>your_robot_namespace</robotNamespace>
      <controlPeriod>the_control_period_you_wish</controlPeriod> <!-- do not create a field if you wish to use Gazebo's control period -->
      <port>port_to_initialize_naoqi</port>
      <modelType>the_model_you_want</modelType>
    </plugin>
  </gazebo>
```

**NOTE that sensor names must be as they are in nao.xacro file..**

Notes/Limitations
-----------------
* Tutorials will become available as soon as possible.
* Cameras, IMU, sonars and FSRs integration is completed, but in beta/testing stage.
* **Fingers/Hand joints are not simulated yet..**
* *Integration for LED, IR, Bumper, Tactile and Audio hardware is not available and is not on my agenda*. So, **feel free to contribute in that direction**.
* I have tested the plugin for V40 and V50 robot models! Using it with older versions should work, but isn't tested!
* **NAO now walks!!** Thanks to @jlanca ..

Future Work
------------
* **Integrate Fingers/Hand joints.**
* Investigate if we can bind simulated NAOqi's time with Gazebo's simulated time.
* Enable stiffness by default when simulation is started.
* Integration of more sensors.
* **PID values need tuning!!!**
* .....

License
----

BSD


Copyright (c) 2014-2015, **Konstantinos Chatzilygeroudis**

[ros]: http://www.ros.org
[gazebo]: http://gazebosim.org
[ros control]: http://wiki.ros.org/ros_control
[nao meshes]: https://github.com/ros-naoqi/nao_meshes
