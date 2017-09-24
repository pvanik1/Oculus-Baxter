# Oculus-Baxter
Oculus Rift + Touch teleoperation of the Baxter robot using the ZED camera. Implements Stereolabs Zed Oculus Viewer:
https://github.com/stereolabs/zed-oculus

<a href="http://www.youtube.com/watch?feature=player_embedded&v=S6BM3BfwyAY
" target="_blank"><img src="http://img.youtube.com/vi/S6BM3BfwyAY/0.jpg" 
alt="YouTube video of the project" width="360" height="240" border="10" /></a> 

### Application files:

Oculus_ZED_Baxter - Windows-side application (Visual Studio) for outputting ZED video and publishing Oculus poses.
   oculus - Ubuntu-side application (Catkin package) for processing pose data and issuing Baxter joint commands.
   dependencies - files required for the Windows application that need to be linked in Visual Studio. See below for version details.

### Additional redundant files:

Oculus_ROS - standalone Visual Studio application for displaying tracking data.
   zed-oculus-master - standalone Visual Studio application for outputting ZED video (Stereolabs' ZED viewer)

### How to run the system
Assuming Oculus sensors are set up and Oculus Home is running:

1. On Ubuntu, run a Baxter shell by navigating to your ROS/Catkin workspace and running ./baxter.sh. 
2. In a Baxter shell, run $ roslaunch rosserial_server socket.launch
3. In a second Baxter shell, run $ rosrun oculus main.py (oculus is the name of the catkin package). This will initialise Baxter.
4. On Windows, launch the application (preferably in Visual Studio to allow quick tweaks - make sure to run in x64 Release configuration). The application will start after an initial sleep period that serves as a delay so the operator has enough time to put on the Rift.
5. Put on the Rift and equip the Touch controllers. Keep looking straight ahead until you see the ZED cam video. Do not change position when using the application.
6. Quit by stopping the processes in both terminals with ctrl+c. Rosserial socket terminal does not need to be restarted 

### How to use the system:
- press and hold index trigger buttons to control the grip of the grippers 
- press and hold A/X buttons to activate Roll mode, where only roll rotation of the operator's wrist is replicated
- large, sweeping movements with elbows extended are most suitable for correct pose replication
- turn your head slowly and keep it tilted slightly donwards to prevent disorientation and motion sickness
- make sure the length of the operator's arms is set in presets.py

#### Known issue: 
Entire arm can sometimes get stuck during roll mode, once desired rotation exceeds maximum possible rotation. This has only started happening when an error in the Rosserial terminal started appearing (despite restarts). No modifications were made to the code that would cause this error to suddenly appear.

### How to build the system:
1. Install all dependencies.
2. Create a catkin package called oculus with the files in the oculus folder
3. Set the path to the head display image accordingly in presets.py
4. Copy the Oculus_ZED_Baxter and dependencies folders to your desired folder. The VS project is based on Stereolabs' ZED Oculus Viewer, so if there is an issue (e.g. with CMakeLists.txt paths), follow their instructions to create the Viewer application. Then put the additional header and cpp files from Oculus_ZED_Baxter to the ZED Viewer folders (\include, \src).
5. Link all the required dependencies in Visual Studio. In Solution explorer, right click on the Oculus_ZED_Baxter project and click "Set as startup project". Right click again and click Properties. Set configuration to Release x64 and do the following under the tabs:

   5.1. VC++ Directories

   &nbsp;&nbsp;&nbsp;5.1.1. Include Directories -> with New Line, put in full paths to:    
           
       \ros_lib
       \OculusSDK\LibOVR\Include 
       \SDL 2.x\include
   &nbsp;&nbsp;&nbsp;5.1.2. Library Directories ->
       
       \LibOVR\lib\Windows\x64\Release\[Visual Studio version you are using]
   5.2. C/C++
   
    &nbsp;&nbsp;&nbsp;5.2.1. Additional include directories ->
      
       \glm
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\include
       E:\...\Oculus-Baxter\Oculus_ZED_Baxter\Oculus_ZED_Baxter\include
       \ZED SDK\dependencies\glew-1.12.0\include
       \ZED SDK\include
   5.3. Linker
   
    &nbsp;&nbsp;&nbsp;5.3.1. General -> Additional library directories ->
      
       \ZED SDK\lib
       \ZED SDK\dependencies\freeglut
       \ZED SDK\dependencies\glew
    &nbsp;&nbsp;&nbsp;5.3.2. Input -> the names of the following lib files, or their system paths:
      
       sl_zed64.lib
       sl_core64.lib
       sl_scanning64.lib
       glu32.lib
       opengl32.lib
       freeglut.lib
       glew32.lib
       [full path]\ZED SDK\dependencies\opencv_3.1.0\x64\vc14\lib\opencv_world310.lib
       SDL2.lib
       SDL2main.lib
       LibOVR.lib
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\lib\x64\cuda.lib
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\lib\x64\cudart.lib
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\lib\x64\nppc.lib
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\lib\x64\nppi.lib
       C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\lib\x64\npps.lib
      
### System dependencies with versions used 
a) Ubuntu (16.04)
       
       ROS Kinetic
       Baxter SDK 1.2.0
b) Windows (10 64-bit)
Developed in Visual Studio 2015 Enteprise with update 3 (64-bit)
       
       Oculus SDK 1.15 https://developer.oculus.com/downloads/package/oculus-sdk-for-windows/1.8.0/
       GLM 0.9.4 https://glm.g-truc.net/0.9.8/index.html
       Rosserial Windows http://wiki.ros.org/rosserial_windows/
       ZED SDK 2.1 https://www.stereolabs.com/developers/
       CUDA 8 with patch 2 https://developer.nvidia.com/cuda-downloads
       SDL 2.0.5 https://www.libsdl.org/download-2.0.php
