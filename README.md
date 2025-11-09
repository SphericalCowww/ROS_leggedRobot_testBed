# Testing for Legged Robot using ROS2

Goal: quadruped robot dog

## Hardware Connections

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| USB interface + power hub | <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> + <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">power hub board</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| servo motor | <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 [N.m] (at 12.0 [V], 1.4 [A], 1.071 [Nm/A]) |
| power supply | SMPS | 4? | 12V 5A each | 
| battery | 18650 | 6 | 3.6-3.7V 1.2-3.5A each; need 2 parallel of 4 in series | 
| controller | <a href="https://emanual.robotis.com/docs/en/parts/controller/opencr10/">OpenCR 1.0</a> | 1 | This is not necessary if a Rasp Pi is already in use |

#### XL430-W250-T dimensions

  * general dimension: 28.5 x 46.5 x 34 [WxHxD mm]
  * screw size: M2 on rotor plate, M2.5 screws on frame corners, M2.6 on frame surfaces
  * <a href="https://en.robotis.com/service/downloadpage.php?ca_id=70">CAD download</a>
  * bracket reference: <a href="https://grabcad.com/library/openmanipulator-x-frame-set-rm-x52-1">OpenManipulator-X</a>

#### 3D CAD dimensions

  * using 3M screws overall, as they are much more common. Piece thickness will be 4mm, except when a 3M screw is required on the thickness axis, then it's 6mm. For aperture dimension, 1.7mm radius for clearance, and 1.25mm for tapped without heated insert.

### setting the servo IDs

Connecting servo to U2D2 according to <a href="https://www.youtube.com/watch?v=FIj_NULYOKQ">YouTube</a>:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/basicConnection_DYNAMIXEL.png" width="200">

Use the following App <a href="https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/">DYNAMIXEL Wizard 2.0</a> to change the servo ID (default 1) to, say, ID 10. Note that if any of the servos have the same ID, they will NOT show up in the scan. The servos also all initially come with an ID of 1, so they must be connected one by one to U2D2 to change their IDs accordingly.

    connect U2D2 to computer => open DYNAMIXEL Wizard 2.0
    # Options 
    ## Select protocol to scan => Protocol 2.0 (only)
    ## Select port to scan => which ever port is connected
    ## Select baudrate to scan => 57600 bps (only) => OK
    # Scan
    # 7 ID => 11 ID 11 (on the right) => Save (may need to scroll down) 

Can also test out the servo:

    # LED (top right toggle)
    # Torque (top right toggle) => set the value for motion (can select Velocity/Position mode)

## Interfacing with ROS2

### installing the <a href="https://github.com/ROBOTIS-GIT/DynamixelSDK">dynamixel-sdk</a> and  <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench">dynamixel-workbench</a>

Following <a href="https://github.com/SphericalCowww/ROS_init_practice">github</a> to install ROS. To install drivers for Dynamixel, 

    sudo apt install ros-jazzy-dynamixel-sdk* ros-jazzy-dynamixel-hardware* ros-jazzy-dynamixel-workbench*
    dpkg -l | grep dynamixel
    ros2 pkg list | grep dynamixel

Then copy <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/main/dynamixel_workbench_toolbox/examples/src">src</a> directly under ``/src/my_toolbox_dynamixel_workbench``. And for every ``.cpp`` file, change the following line:

    #include <DynamixelWorkbench.h>

To, 

    #include <cstdlib>
    #include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

Then ``cd ROS_leggedRobot_testBed`` and build,

    colcon build
    source install/setup.bash

Connect U2D2 to Rasp Pi USB port: 

    sudo dmesg | tail -n 20
    # look for:
    ## usb 2-2: Detected FT232H
    ## usb 2-2: FTDI USB Serial Device converter now attached to ttyUSB0
    ls -l /dev/ttyUSB*
    sudo chmod a+rw /dev/ttyUSB0                   # required everytime after reconnection
    ros2 run my_toolbox_dynamixel_workbench model_scan /dev/ttyUSB0 57600
    ros2 run my_toolbox_dynamixel_workbench position /dev/ttyUSB0 57600 11 0.5

### testing the driver in ROS2

    colcon build
    source install/setup.bash
    sudo chmod a+rw /dev/ttyUSB0                   
    ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_channel0_xl430 
    ps -ef | grep testRaspPi5_dynamixel_u2d2_channel0_xl430                 # to kill it before it ends
    ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_oneleg_xl430 

### testing MoveIt with Gazebo

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.gazebo.launch.with_commander.py
    ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/MaRobotArmPoseTarget "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"

### testing the driver with ros2_control and MoveIt
Under ``ma_robot.ros2_control.xacro``, switch ``<plugin>mock_components/GenericSystem</plugin-->`` to ``<plugin>ma_robot_namespace::HardwareInterfaceU2D2_ma_robot</plugin>``. The latter plugin type can be found at the bottom of ``src/my_robot_firmware/hardware_interface_ma_robot_dynamixel_u2d2_xl430.xml``.

  * src/my_robot_**bringup**/launch/**ma_robot.with_commander.launch.py**
    * src/my_robot_description/urdf/ma_robot.urdf.xacro
      * src/my_robot_description/urdf/common_properties.xacro
      * src/my_robot_description/urdf/ma_robot_arm.xacro.xacro
      * src/my_robot_description/urdf/ma_robot_gripper.xacro.xacro
      * src/my_robot_**description**/urdf/**ma_robot.ros2_control.xacro**
        * src/my_robot_firmware/hardware_interface_ma_robot_dynamixel_u2d2_xl430.xml
        * src/my_robot_**firmware**/include/my_robot_firmware/**hardware_interface_ma_robot_dynamixel_u2d2_xl430.hpp**
    * src/my_robot_**bringup**/config/**ma_robot_controllers.yaml**
      * joint_trajectory_controller/JointTrajectoryController
    * src/**ma_robot_moveit_config**/launch/**move_group.launch.py**
      * src/ma_robot_moveit_config/config...
    * src/my_robot_commander/src/ma_robot_commander.cpp
    * src/my_robot_description/rviz/ma_robot.urdf_config.rviz

Then run the following:

    colcon build
    source install/setup.bash
    sudo chmod a+rw /dev/ttyUSB0
    ros2 launch my_robot_bringup ma_robot.with_commander.launch.py
    ros2 topic info /arm_set_name
    ros2 topic pub -1 /arm_set_named example_interfaces/msg/String "{data: "arm_pose1"}"
    ros2 topic info /arm_set_joint
    ros2 topic pub -1 /arm_set_joint example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
    ros2 topic info /arm_set_pose
    ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/MaRobotArmPoseTarget "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"
    ros2 topic info /gripper_set_open
    ros2 topic pub -1 /gripper_set_open example_interfaces/msg/Bool "{data: false}"

### Quadruped: CAD Model

One leg assembly needs 3 servos to ensure all 3 degrees of freedom for each of the 4 leg tips:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/CAD/zAssembly1Leg.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/CAD/zAssembly4Leg.png" width="350">

## References:
- AstroSam, I Made a Robot Dog (2024) (<a href="https://www.youtube.com/watch?v=XvKlplncafQ">YouTube</a>)
