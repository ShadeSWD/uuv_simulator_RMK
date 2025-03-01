# `uuv_simulator`: Unmanned Underwater Vehicle (UUV) simulation with Gazebo

The **Unmanned Underwater Vehicle Simulator** is a set of packages that include plugins and ROS applications that allow simulation of underwater vehicles in [Gazebo](http://gazebosim.org/). 

If you are using this simulator for your publication, please cite:

```
@inproceedings{Manhaes_2016,
	doi = {10.1109/oceans.2016.7761080},
	url = {https://doi.org/10.1109%2Foceans.2016.7761080},
	year = 2016,
	month = {sep},
	publisher = {{IEEE}},
	author = {Musa Morena Marcusso Manh{\~{a}}es and Sebastian A. Scherer and Martin Voss and Luiz Ricardo Douat and Thomas Rauschenbach},
	title = {{UUV} Simulator: A Gazebo-based package for underwater intervention and multi-robot simulation},
	booktitle = {{OCEANS} 2016 {MTS}/{IEEE} Monterey}
}
```

In you are willing to contribute to this package, please check the instructions in [CONTRIBUTING](https://github.com/uuvsimulator/uuv_simulator/blob/master/CONTRIBUTING.md).

# Features

> **Gazebo/ROS plugins**
  
- Implementation of Fossen's equations of motion for underwater vehicles
- Thruster modules with implementations for thruster's angular velocity to output thrust force based on [`Yoerger el al., 1990`](http://dx.doi.org/10.1109/48.107145) and [`Bessa et al., 2006`](http://www.abcm.org.br/symposium-series/SSM_Vol2/Section_IX_Submarine_Robotics/SSM2_IX_01.pdf)
- Lift and drag plugin for simulation of fins
- Simulation of 3D current velocity models (constant or based on first-order Gauss-Markov processes)
- Sensor plugins

> **Controllers**

- For AUVs
    - [`casadi`](https://web.casadi.org/)-based effort allocation algorithm 
    - Geometric tracking PD controller
- For ROVs
    - Thruster manager with computation of the thruster allocation matrix based on the thruster frames available in `/tf`
    - Model-based feedback linearization controller ([`Fossen, 2011`](https://www.wiley.com/en-us/Handbook+of+Marine+Craft+Hydrodynamics+and+Motion+Control-p-9781119991496))
    - Nonlinear PID controller ([`Fossen, 2011`](https://www.wiley.com/en-us/Handbook+of+Marine+Craft+Hydrodynamics+and+Motion+Control-p-9781119991496))
    - Non-model-based sliding mode controller ([`García-Valdovinos el al., 2014`](https://journals.sagepub.com/doi/full/10.5772/56810) and [`Salgado-Jiménez et al., 2011`](http://cdn.intechopen.com/pdfs/15221.pdf))
    - PD controller with restoration forces compensation 
    - 6-DOF PID controller
    - Singularity-free tracking controller ([`Fjellstad and Fossen, 1994`](https://ieeexplore.ieee.org/abstract/document/411068))
- Teleoperation nodes for AUVs and ROVs

> **Gazebo world models**

- Ocean wave shaders for wave animation
- Scenarios from the SWARMs project demonstration locations (e.g. Mangalia, Romania and Trondheim, Norway)
- Subsea BOP panel for manipulation tasks

> **Vehicle models**

- Work-class ROV `rexrov` based on the model presetend in [`Berg, 2012`](https://brage.bibsys.no/xmlui/handle/11250/238170?locale-attribute=no)
- [`eca_a9`](https://github.com/uuvsimulator/eca_a9)
- [`lauv_gazebo`](https://github.com/uuvsimulator/lauv_gazebo)
- [`desistek_saga`](https://github.com/uuvsimulator/desistek_saga)
- [`rexrov2`](https://github.com/uuvsimulator/rexrov2)
  
# Installation

If you don't have the ROS workspace yet, you should run the following and then clone the uuv_simulator package in the ~/catkin_ws/src folder
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
Be sure to install catkin tools package by following the installation instructions on the catkin tools documentation page. After the installation, initialize the catkin workspace
```
cd ~/catkin_ws
catkin init
```
You can then clone the UUV simulator into your src folder
```
cd ~/catkin_ws/src
git clone https://github.com/uuvsimulator/uuv_simulator.git
```
Configure the environment variables by adding the following lines in ~/.bashrc (replace kinetic with the ROS version you are using).
```
source /usr/share/gazebo-7/setup.sh
source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
```
After saving these changes, remember to source the .bashrc by either typing
```
source ~/.bashrc
```

# Quick start

Start an empty underwater environment using either
```
roslaunch uuv_gazebo_worlds empty_underwater_world.launch
```
or
```
roslaunch uuv_gazebo_worlds ocean_waves.launch
```
Spawn the remotely operated vehicle RexROV (find the robot description files in uuv_descriptions) as follows
```
roslaunch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=-20 namespace:=rexrov
```
for which mode stands for the configuration of the vehicle to be used. It is important to create the vehicles under a unique namespace to allow simulation of multiple vehicles in the same scenario.

You can start a velocity controller with a joystick teleoperation node as
```
roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0
```
In this case model_name refers to the vehicle model, which can be different from the namespace. It is a necessary parameter to load the correct controller and thruster allocation matrix coefficients. The joystick ID is already set zero as default. To find the correct joystick index, you can install and run jstest-gtk.

To use keyboard teleoperation run:
```
roslaunch uuv_control_cascaded_pid key_board_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0
```
In this case model_name refers to the vehicle model, which can be different from the namespace. It is a necessary parameter to load the correct controller and thruster allocation matrix coefficients. The joystick ID is already set zero as default. To find the correct joystick index, you can install and run jstest-gtk.

To use keyboard teleoperation run:
```
roslaunch uuv_control_cascaded_pid key_board_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0
```
In this case model_name refers to the vehicle model, which can be different from the namespace. It is a necessary parameter to load the correct controller and thruster allocation matrix coefficients. The joystick ID is already set zero as default. To find the correct joystick index, you can install and run jstest-gtk.

To launch Nonius teleoperation and thresters controller node run:
```
roslaunch thrusters_controllers rexrov_controller.launch 
```

# Purpose of the project

This software is a research prototype, originally developed for the EU ECSEL
Project 662107 [SWARMs](http://swarms.eu/).

The software is not ready for production use. However, the license conditions of the
applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software
fulfills your requirements and adjust it according to any applicable safety
standards (e.g. ISO 26262).

