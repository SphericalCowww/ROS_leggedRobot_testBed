# Testing for Legged Robot using ROS2

Goal: quadruped robot dog

## Hardware Connections

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| USB communication interface | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| communication/power hub | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">U2D2 power hub board</a> | 2 | Operating voltage	3.5-24V withg a maximum current	of 10A |
| servo motor | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 N*m (at 12.0V, 1.4A, 1.071 Nm/A) |
| battery | ZYGY <a href="https://www.amazon.de/dp/B0BB6RMM5Q">11.1V 2000mAh</a> | 2 | already includes protection. Need adapters for: T-plug => XT60 Male => DC 5.5mm x 2.5mm Male | 
| battery | Palogreen <a href="https://www.amazon.de/dp/B0DZ65LMPT">12.6V DC Akku 2800mAh</a> | 4 | already includes protection. One for powering the rasp pi 5, and several backups | 
| power supply | SMPS | 1 | 12V/5A to test 1 leg | 
|  |  |  |  | 
| battery | <a href="https://secure.reichelt.com/de/en/industrial_cell_18650_3_6_v_2600_mah_unprotected_pack_of_2-p261054.html">SON 18650 VTC5A</a> | 9 | Operating voltage 3.6-3.7V with a discharge max of 10A each; need 3 parallel of 3 in series | 
| battery management system | <a href="https://powercells.de/3s-40a-12-6v-cell-li-on-akku-batterie-schutz-platine-bms-pcb-lithium-protection.html?language=en">3S 40A 12.6V BSM PCB</a> | 3 | plan to use one for each of the 3-series battery pack | 
| controller | <a href="https://emanual.robotis.com/docs/en/parts/controller/opencr10/">OpenCR 1.0</a> | 1 | This is not necessary if a Rasp Pi is already in use |

#### XL430-W250-T dimensions

  * general dimension: 28.5 x 46.5 x 34 [WxHxD mm]
  * screw size: M2 on rotor plate, M2.5 screws on frame corners, M2.6 on frame surfaces
  * <a href="https://en.robotis.com/service/downloadpage.php?ca_id=70">CAD download</a>
  * bracket reference: <a href="https://grabcad.com/library/openmanipulator-x-frame-set-rm-x52-1">OpenManipulator-X</a>

#### 3D CAD dimensions

  * using 3M screws overall, as they are much more common. Piece thickness will be 4mm, except when a 3M screw is required on the thickness axis, then it's 6mm. For aperture dimension, 1.7mm radius for clearance, and 1.25mm for tapped without a heated insert.

### Power System

  * The daisy chain is recommended to chain 4 or fewer servos to avoid delay
  * Plan to use 2 U2D2 power hub boards controlled by one U2D2. Each hub can be powered independently and controlled by a single U2D2 as long as they share the TTL connection. This requires a Y-cable from U2D2 to the 2 hubs; may need to custom-connect two <a href="https://emanual.robotis.com/docs/en/dxl/x/xl320/#connector-information">MOLEX 51065-0300</a> cables to make a Y-cable.
  * Plan to power the hubs using their SMPS DC jacks. Then use the molex/screw terminal as an output to power the Raspberry Pi 5 via a 12V-to-5V DC-DC converter.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystem.png" width="500"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/powerSystemPhoto.png" width="200">

When testing, the 11.1V battery can be teed to power both Raspberry Pi 5 and 3 servos. However, Raspberry Pi 5 does show low power when also turning on the monitor with HDMI connection.



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

### Quadruped: CAD Model and Physical Assembly

One leg assembly needs 3 servos to ensure all 3 degrees of freedom for each of the 4 leg tips:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/src/my_robot_description/mesh/CADv0/zAssembly1Leg.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/src/my_robot_description/mesh/CADv0/zAssembly4Leg.png" width="350">

The current design has many flaws, but here is the first assembly:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly1Leg1.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly1Leg2.png" width="300">

[Video demo1](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/assembly1Leg1.mp4), [video demo2](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/assembly1Leg2.mp4)

## Installing the <a href="https://github.com/ROBOTIS-GIT/DynamixelSDK">dynamixel-sdk</a> and  <a href="https://github.com/ROBOTIS-GIT/dynamixel-workbench">dynamixel-workbench</a>

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

## Testing ROS2 interface with a simulated robot arm

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

## Testing ROS2 interface with 1 leg

### launch urdf

Then run the following:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_description my_robot.rviz.launch.xacro.py
    # if no config loaded
    ## Fixed Frame: base_link
    ## Add: RobotModel
    ## RobotModel: Description Topic: /robot_description

[Video demo](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/rViz1Leg.mp4)

### moveit2 setup assistance with an arm

Launch the MoveIt assistance:

    sudo apt update
    sudo apt install
    colcon build
    source install/setup.bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    # Create New Moveit Configuration Package (or edit if the configuration files already exist)
    # Browse => src/my_robot_description/urdf/my_robot.urdf.xacro => Load Files
    # Start Screen: can toggle visual/collision
    # Self-Collisions => Generate Collision Matrix: removes all never-in-contact and adjacent collisions
    # Virtual Joints => Add Virtual Joint => Virtual Joint Name: virtual_joint => Parent Frame Name: world => Joint Type: floating => Save
    # Planning Groups => Add Group => Group Name: leg1 => Kinametic Solver: kdl_kinematics_plugin => Add Joints => 
    ## choose with right arrow "servo1_servo1_padding", "servo2_servo2_padding", "servo3_calfFeet", and "calfFeet_calfSphere" => Save
    # Robot Poses => Add Pose => all joints at 0 => Pose Name: home => Save: can add a few other ones for debugging
    # ros2_control URDF Model => position for Command Interfaces and State Interfaces => Add interfaces
    # ROS2 Controllers => Auto Add JointTrajectoryController
    # Moveit Controllers => Auto Add FollowJointsTrajectory
    # Author Information => add anything (e.g. "my_robot", "my_robot@gmail.com"), otherwise bugged
    # Configuration Files => Browse: src/my_robot_moveit_config/ => Generate Package: double check if files are generated => Exit Setup Assistant

Fix the following file:

    # src/my_robot_moveit_config/config/joint_limits.yaml => max_velocity: 1.0, has_acceleration_limits: true, max_acceleration: 1.0 (need to be float)
    # src/my_robot_moveit_config/config/moveit_controllers.yaml => add the following under leg1_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true
    # src/my_robot_moveit_config/config/initial_positions.yaml => servo1_servo1_padding: 3.14, servo2_servo2_padding: 3.14, servo3_calfJoint: 3.14
    # src/my_robot_moveit_config/config/my_robot.srdf => include only the following in <group name="leg1">:
    ## <group name="leg1">
    ##     <chain base_link="base_link" tip_link="calfSphere"/>
    ## </group>

Launch the demo:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_moveit_config demo.launch.py
    # ignore: [move_group-3] [ERROR] [1758361830.007872451] [move_group.moveit.moveit.ros.occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates
    # ignore: [rviz2-4] [ERROR] [1758361834.128908606] [moveit_143394722.moveit.ros.motion_planning_frame]: Action server: /recognize_objects not available
    # MotionPlanning:
    ## Planning Group: leg1
    ## Goal State: pose1
    ## Plan
    ## Execute
## References:
- AstroSam, I Made a Robot Dog (2024) (<a href="https://www.youtube.com/watch?v=XvKlplncafQ">YouTube</a>)
