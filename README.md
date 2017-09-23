# Oculus-Baxter
Oculus Rift + Touch teleoperation of the Baxter robot using the ZED camera. Implements Stereolabs Zed Oculus Viewer:
https://github.com/stereolabs/zed-oculus

Application files:
Oculus_ZED_Baxter - Windows-side application (Visual Studio) for outputting ZED video and publishing Oculus poses.
oculus - Ubuntu-side application (ROS Baxter terminal) for processing pose data and issuing Baxter joint commands.
dependencies - files required for the Windows application that need to be linked in Visual Studio. See below for version details.

Additional redundant files:
Oculus_ROS - standalone Visual Studio application for displaying tracking data.
zed-oculus-master - standalone Visual Studio application for outputting ZED video (Stereolabs' ZED viewer)
