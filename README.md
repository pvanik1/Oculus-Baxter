# oculusros
Oculus Rift + Touch (Windows 10) telemetric control of the Baxter robot on ROS (Ubuntu 16.04). Publishes data from a Windows pc to a Ubuntu Baxter workstation. Built using Visual Studio 2015. Requirements: Oculus Rift + Touch, Baxter robot / Gazebo simulator.

The only relevant folders are Oculus and Oculus_ROS, the rest are old test versions.
Folders:

myoculus2 - first simple version of getting data from Oculus Rift to console

myoculus3_euler_loop - working version of printing full HMD and Oculus Touch data to console

test01 - working rosserial_windows application creating a ROS node on Windows

Oculus_ROS - working windows-side application creating a ROS node publishing all orientation and position data.

oculus - ubuntu-side catkin src package

Final version and instructions will be published by 10/2017.
