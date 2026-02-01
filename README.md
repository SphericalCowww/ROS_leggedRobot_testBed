# Testing for Legged Robot using ROS2

Goal: quadruped robot dog

## Hardware Connections

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| controller | <a href="https://www.raspberrypi.com/products/raspberry-pi-5/">rasp pi 5</a> | 1 | 16Gb RAM | 
| DC-DC step-down convertor | Hailege <a href="https://www.amazon.de/Hailege-Module-Step-Down-Supply-Converter/dp/B07XFMMY1F">24V/12V to 5V/5A</a> | 1 | USB Port port to rasp pi 5,  DC 5.5mm x 2.5mm Male to battery | 
| USB communication interface | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> | 1 | Can control 12 servo in daisy chain if properly powered |
| communication/power hub | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/">U2D2 power hub board</a> | 2 | Operating voltage	3.5-24V withg a maximum current	of 10A |
| servo motor | DYNAMIXEL <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | Max stall torque: 1.5 N*m (at 12.0V, 1.4A, 1.071 Nm/A) |
| battery | ZYGY <a href="https://www.amazon.de/dp/B0BB6RMM5Q">11.1V 2000mAh</a> | 2 | already includes protection. Need adapters for: T-plug => XT60 Male => DC 5.5mm x 2.5mm Male | 
|  |  |  |  | 
| battery | Palogreen <a href="https://www.amazon.de/dp/B0DZ65LMPT">12.6V DC Akku 2800mAh</a> | - | already includes protection. One for powering the rasp pi 5, and several backups | 
| power supply | SMPS | - | 12V/5A to test 1 leg | 
| battery | <a href="https://secure.reichelt.com/de/en/industrial_cell_18650_3_6_v_2600_mah_unprotected_pack_of_2-p261054.html">SON 18650 VTC5A</a> | - | Operating voltage 3.6-3.7V with a discharge max of 10A each; need 3 parallel of 3 in series | 
| battery management system | <a href="https://powercells.de/3s-40a-12-6v-cell-li-on-akku-batterie-schutz-platine-bms-pcb-lithium-protection.html?language=en">3S 40A 12.6V BSM PCB</a> | - | plan to use one for each of the 3-series battery pack | 
| controller | <a href="https://emanual.robotis.com/docs/en/parts/controller/opencr10/">OpenCR 1.0</a> | - | This is not necessary if a Rasp Pi is already in use |

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

When testing, the 11.1V battery can be teed to power both Raspberry Pi 5 and 3 servos. However, rasp pi 5 does show low power when also turning on the monitor with HDMI connection.

### setting the servo IDs

Connecting servo to U2D2 according to <a href="https://www.youtube.com/watch?v=FIj_NULYOKQ">YouTube</a>:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/basicConnection_DYNAMIXEL.png" width="200">

Use the following App <a href="https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/">DYNAMIXEL Wizard 2.0</a> to change the servo ID (default 1) to, say, ID 10. Note that if any of the servos have the same ID, they will NOT show up in the scan. The servos also all initially come with an ID of 1, so they must be connected one by one to U2D2 to change their IDs accordingly.

    connect U2D2 to computer => open DYNAMIXEL Wizard 2.0
    # Options 
    ## Select protocol to scan => Protocol 2.0 (only)
    ## Select port to scan => which ever port is connected
    ## Select baudrate to scan => 57600 bps and 2000000bpt => OK
    ## Select ID range to scan => End => 50
    ## OK
    # Scan
    # Item 
    ## (Address 7) ID => ID 11 (on the right) => Save (may need to scroll down) 
    ## (Address 8) Baud Rate (Bus) => 2Mbps (on the right) => Save (may need to scroll down) 
    ## (Address 9) Return Delay Time => 0 (on the right) => Save (may need to scroll down) 
    ## (Address 68) Status Return Level => 2 (on the right) => Save (may need to scroll down) 

Can also test out the servo:

    # LED (top right toggle)
    # Torque (top right toggle) => set the value for motion (can select Velocity/Position mode)

### Quadruped: CAD Model and Physical Assembly

One leg assembly needs 3 servos to ensure all 3 degrees of freedom for each of the 4 leg tips:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/src/my_robot_description/mesh/CADv0/zAssembly1Leg.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/src/my_robot_description/mesh/CADv0/zAssembly4Leg.png" width="350">

The current design has many flaws, but here is the first assembly:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly1Leg1.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly1Leg2.png" width="300">

[Video demo1](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/assembly1Leg1.mp4), [video demo2](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/assembly1Leg2.mp4)

Here is the assembly with the hardware control system: rasp pi 5, DC-DC step-down convertor, U2D2, battery pack, and the existing 1 leg. DO NOT wrap the USB signal cable with the power cables; it would create interference.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly2MainBoard.png" width="300">

Here is the full assembly. Also, update the unnecessary bearing at the connection of the universal bearings. Moreover, be warned that standing up requires specific sequences to prevent overloading the servos; the robot has a total weight of roughly 2kg.

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly4Legs1.png" width="300"> <img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/assembly4Legs2.png" width="350">

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

Update USB port latency to 1 ms. Note that this USB signal is delicate, use as short and as high quality of the USB cable as possible.: 

    lsusb
    # look for: Bus 002 Device 003: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC
    sudo vim /etc/udev/rules.d/99-dynamixel-latency.rules
    ---------- add: 
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTR{device/latency_timer}="1", SYMLINK+="ttyU2D2", MODE="0666", GROUP="dialout"
    # ATTR{device/latency_timer}="1": Sets the 1ms latency (The "Sync Read" fix).
    # SYMLINK+="ttyU2D2": (Optional but helpful) This creates a static name for your device. You can now use /dev/ttyU2D2 in your code instead of /dev/ttyUSB0, so it won't break if you plug in another USB device.
    # MODE="0666": Allows your ROS node to access the port without needing sudo.
    ---------- 
    sudo vim /etc/udev/rules.d/99-ftdi.rules
    ---------- add: 
    SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}==6014, MODE="0666", GROUP="dialout"
    SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0666", GROUP="dialout"
    # This gives the correct permission to the USB port in question, otherwise, whenever reconnected, needs to do: sudo chmod a+rw /dev/ttyUSB0
    ---------- 
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ls -l /dev/ttyU2D2
    # if doesn't exist, do: sudo apt remove brltty
    cat /sys/class/tty/ttyUSB0/device/latency_timer
    # which should show 1 for 1 ms

Update to run the firmware with proper permissions to avoid latency:

    # without sudo, we will see the following WARNING in ros2 launch:
    ## [ros2_control_node-2] [WARN] [1766867979.090889912] [controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1> (Operation not permitted). See [https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] for details on how to enable realtime scheduling.
    # with sudo, we can do the following:
    ## sudo bash -c "source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 launch my_robot_bringup my_robot.with_commander.launch.py"
    # however, eventually we want to run without sudo in case it messes up with other permissions:
    sudo addgroup realtime
    sudo usermod -a -G realtime $USER
    sudo vim /etc/security/limits.d/realtime.conf:
    ----------  add:
    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock unlimited
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock unlimited
    ---------- 
    sudo reboot
    # 


## Testing ROS2 interface with a simulated robot arm

### testing the driver in ROS2

    colcon build
    source install/setup.bash
    ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swing_xl430
    ps -ef | grep testRaspPi5_dynamixel_u2d2_leg1swing_xl430                 # to kill it before it ends
    # only when dynamixels are not connected to into a leg: 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_channel0_xl430 
    ## ros2 run my_robot_firmware testRaspPi5_dynamixel_u2d2_leg1swipe_xl430

### testing MoveIt with Gazebo

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.gazebo.launch.with_commander.py
    ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/MaRobotArmPoseTarget "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"

### testing the driver with ros2_control and MoveIt
Under ``ma_robot.ros2_control.xacro``, switch ``<plugin>mock_components/GenericSystem</plugin-->`` to ``<plugin>ma_robot_namespace::HardwareInterfaceU2D2_ma_robot</plugin>``. The latter plugin type can be found at the bottom of ``src/my_robot_firmware/hardware_interface_ma_robot_dynamixel_u2d2_xl430.xml``. Then run the following:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup ma_robot.with_commander.launch.py
    ros2 topic info /arm_set_name
    ros2 topic pub -1 /arm_set_named example_interfaces/msg/String "{data: "arm_pose1"}"
    ros2 topic info /arm_set_joint
    ros2 topic pub -1 /arm_set_joint example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"
    ros2 topic info /arm_set_pose
    ros2 topic pub -1 /arm_set_pose my_robot_interface/msg/MaRobotArmPoseTarget "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, use_cartesian_path: false}"
    ros2 topic info /gripper_set_open
    ros2 topic pub -1 /gripper_set_open example_interfaces/msg/Bool "{data: false}"

## Launch ROS2 interface with 1 leg

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

### moveit2 setup assistance with a leg

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
    # Virtual Joints => Add Virtual Joint => Virtual Joint Name: virtual_joint => Parent Frame Name: world => Joint Type: fixed => Save
    ## no need if already defined in urdf. Can always comment out afterwards in /src/my_robot_moveit_config/config/my_robot.srdf 
    # Planning Groups => Add Group => Group Name: leg1 => Kinametic Solver: kdl_kinematics_plugin => Add Joints => 
    ## choose with right arrow "servo1_servo1_padding", "servo2_servo2_padding", "servo3_calfFeet", and "calfFeet_calfSphere" => Save
    # Robot Poses => Add Pose => all joints at 0 => Pose Name: home => Save: can add a few other ones for debugging
    # ros2_control URDF Model => position for Command Interfaces and State Interfaces => Add interfaces
    # ROS2 Controllers => Auto Add JointTrajectoryController
    # Moveit Controllers => Auto Add FollowJointsTrajectory
    # Author Information => add anything (e.g. "my_robot", "my_robot@gmail.com"), otherwise bugged
    # Configuration Files => Browse: src/my_robot_moveit_config/ => Generate Package: double check if files are generated => Exit Setup Assistant

Fix the following file:

    # src/my_robot_moveit_config/config/joint_limits.yaml => max_velocity: 20.0, has_acceleration_limits: true, max_acceleration: 10.0 (need to be float)
    # src/my_robot_moveit_config/config/moveit_controllers.yaml => add the following under leg1_controller: 
    ## action_ns: follow_joint_trajectory
    ## default: true
    # src/my_robot_moveit_config/config/initial_positions.yaml => servo1_servo1_padding: 3.14, servo2_servo2_padding: 3.14, servo3_calfJoint: 3.14
    # src/my_robot_moveit_config/config/my_robot.srdf => include only the following in <group name="leg1">:
    ## <group name="leg1">
    ##     <chain base_link="base_link" tip_link="calfSphere"/>
    ## </group>
    # src/my_robot_moveit_config/config/kinematics.yaml => replace with the following:
    ##leg1:
    ##  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    ##  kinematics_solver_search_resolution: 0.005
    ##  kinematics_solver_timeout: 0.05
    ##  kinematics_solver_attempts: 3
    ##  position_only_ik: True        # this one is important because the leg does NOT care about the orientation of the end effector

#### launch the demo:

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

Note that to move the motion wheel in rViz:

    # toggle: Approx IK Soluations
    # toggle if needed: MotionPlanning => Planned Path => Loop Animation
    # toggle if needed: Use Cartesian Path 

#### launch with a proper launch file:

    mv src/my_robot_moveit_config/config/ros2_controllers.yaml src/my_robot_bringup/config/my_robot_controllers.yaml
    # change the following line if needed in my_robot_controllers.yaml
    ## update_rate: 100 # Hz
    mv src/my_robot_moveit_config/config/my_robot.ros2_control.xacro src/my_robot_description/urdf/
    rm src/my_robot_moveit_config/config/my_robot.urdf.xacro
    # modify the following line in my_robot.ros2_control.xacro:
    ## remove: <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
    ## for all servos, update to: <param name="initial_value">3.14</param> 
    # adding the following line in my_robot.urdf.xacro:
    ## <xacro:include filename="my_robot.ros2_control.xacro" />
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.launch.py
    # NOTE: Sometimes it takes a second try to have everything registered
    # Add => MotionPlanning
    ## Context => Planning Library => ompl
    ## Planning => Goal State: pose1 => Plan => Execute

#### launch with gazebo, command line enabled

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.gazebo.with_commander.launch.py
    ros2 topic pub -1 /leg1_set_named example_interfaces/msg/String "{data: "pose1"}"

#### launch with hardware, command line enabled
Under ``my_robot.ros2_control.xacro``, switch ``<plugin>mock_components/GenericSystem</plugin-->`` to ``<plugin>my_robot_namespace::HardwareInterfaceU2D2_my_robot</plugin>``. 

  * src/my_robot_**bringup**/launch/**my_robot.with_commander.launch.py**
    * src/my_robot_description/urdf/my_robot.urdf.xacro
      * src/my_robot_description/urdf/common_properties.xacro
      * src/my_robot_description/urdf/my_robot_arm.xacro.xacro
      * src/my_robot_description/urdf/my_robot_gripper.xacro.xacro
      * src/my_robot_**description**/urdf/**my_robot.ros2_control.xacro**
        * src/my_robot_firmware/hardware_interface_my_robot_dynamixel_u2d2_xl430.xml
        * src/my_robot_**firmware**/include/my_robot_firmware/**hardware_interface_my_robot_dynamixel_u2d2_xl430.hpp**
    * src/my_robot_**bringup**/config/**my_robot_controllers.yaml**
      * joint_trajectory_controller/JointTrajectoryController
    * src/**my_robot_moveit_config**/launch/**move_group.launch.py**
      * src/my_robot_moveit_config/config...
    * src/my_robot_commander/src/**my_robot_commander.cpp**
      * src/my_robot_interface/msg/MyRobotLeg1PoseTarget.msg
    * src/my_robot_description/rviz/my_robot.urdf_config.rviz

Then run the following:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.with_commander.launch.py
    # on another window
    ros2 topic pub -1 /leg1_set_named example_interfaces/msg/String "{data: "pose1"}"
    ros2 topic pub -1 /leg1_set_joint example_interfaces/msg/Float64MultiArray "{data: [3.14, 3.14, 3.14]}"
    ros2 topic pub -1 /leg1_set_pose my_robot_interface/msg/MyRobotLeg1PoseTarget "{x: -0.092, y: 0.053, z: 0.135, use_cartesian_path: false}" 
    ros2 topic pub -1 /leg1_set_walk example_interfaces/msg/String "{data: "walk1"}" # demo in the .mp4

[Video demo1](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/walkGait0_1Leg.mp4)
    
    # for debugging
    ros2 topic echo /joint_states # use this to track servo positions instead of printing out in src/my_robot_firmware/hardware_interface_my_robot_dynamixel_u2d2_xl430.xml
    ros2 topic hz /joint_states
    ros2 run tf2_tools view_frames
    ros2 param list /move_group | grep kinematics
    ros2 param get /move_group robot_description_kinematics.leg1.kinematics_solver

Note the Cartesian path currently does work due to <a href="https://github.com/moveit/moveit2/issues/3658">moveit issue</a>. For debugging:

    ros2 param get /move_group jump_threshold
    ros2 param list /move_group

#### PILZ planning

Industrial robot arm planning (see <a href="https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html">reference</a>), which is likely too stringent and slow for walk-gait:

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.with_commander.launch.py
    # on another window, first move to the starting position
    ros2 topic pub -1 /leg1_set_pose my_robot_interface/msg/MyRobotLeg1PoseTarget "{x: -0.09, y: 0.01, z: 0.13, use_cartesian_path: false}" 
    ros2 topic pub -1 /leg1_set_walk example_interfaces/msg/String "{data: "walk5"}"

Also, use the following to track the rasp pi cpu temperature in case the planning is overclocking the CPU:

    sudo vcgencmd measure_temp

[Video demo1](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/walkGait1_1Leg.mp4)

#### IPTP/TOTG planning

The usual moveit execution always stops at each pose before starting another. The Iterative Parabolic Time Parameterization (IPTP) or Time Optimal Trajectory Generation (TOTG) on a manually constructed joint trajectory should smooth out the walk-gait. The IK can be taken out of the while loop too, reducing the processing time of repeated motion.

    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.with_commander.launch.py
    # on another window, first move to the starting position
    ros2 topic pub -1 /leg1_set_walk example_interfaces/msg/String "{data: "walk6"}"

Note, turns out IPTP is not supported by the newest moveit2 by default.

[Video demo2](https://raw.githubusercontent.com/SphericalCowww/ROS_leggedRobot_testBed/main/walkGait2_1Leg.mp4)

#### launch with hardware, commander as a lifecycle

    ros2 launch my_robot_bringup my_robot.with_lifecycle.launch.py
    ros2 lifecycle set /my_robot_lifecycle configure
    ros2 lifecycle set /my_robot_lifecycle activate
    ros2 topic pub -1 /leg1_set_named example_interfaces/msg/String "{data: "pose1"}"

## Training with Isaac Sim

### Hardware

Planned hardware spec (<a href="https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html">requirement reference</a>):

  * AMD Ryzen 7 7800X3D - 8 x 4.2 GHz (TurboBoost bis 5.00 GHz, 8 Kerne / 16 Threads, 96MB Cache)
  * NVIDIA GeForce RTX 5080 (16GB GDDR7)
  * 64 GB DDR5 RAM
  * 2000 GB M.2 SSD
  * Gigabit Ethernet LAN, Wi-Fi 6
  * 6x USB 3.2, 4x USB 2.0, 2x HDMI 2.1a, 3x DisplayPort 1.4a, 1x RJ-45, 1x Mikrophon, 1x Kopfhörer, Line-In/ Line-Out/ Mikrofon
  * Xilence Performance A+ M705D => updated to: be quiet! Pure Loop 2 FX 240mm
  * 850 Watt
  * 210 x 430 x 444 mm

### Installation

To install Isaac Sim, follow <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_workstation.html">installation guide</a> and get zip file from <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html">download links</a>:

    sudo apt update
    sudo apt install software-properties-common -y
    sudo add-apt-repository ppa:deadsnakes/ppa -y
    sudo apt update
    sudo apt install python3.11 python3.11-venv python3.11-dev

    python3.11 -m venv isaacsim-env
    source isaacsim-env/bin/activate
    pip install --upgrade pip
    pip install isaacsim[compatibility-check]   # do not install any other packages yet
    isaacsim isaacsim.exp.compatibility_check   # if see orange: Settings => Power => Power Mode => Performance
    deactivate

    sudo snap install code --classic   # installing Visual Studio Code

    mkdir isaacsim
    unzip ~/Downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d isaacsim
    cd isaacsim
    ./post_install.sh
    ./isaac-sim.sh # after the initial opening, an APP "Isaac Sim" actually appears

    # check also APP "NVIDIA X Server Settings" to adjust GPU settings
    sudo apt install nvtop
    nvtop                        # for monitoring GPU

To load the assets locally, follow <a href="https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html">"Assets" guide</a> and get zip file from <a href="https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html">download links (3 parts)</a>:
    mkdir .../Kit/assets
    cd .../Kit/assets
    7z x isaac-sim-assets-complete-5.1.0.zip.001
    vim .../isaacsim/apps/isaacsim.exp.base.kit
    # following: https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html
    # find: 
    ## [settings]
    # add below:
    ## persistent.isaac.asset_root.default = ".../Kit/assets/Isaac/5.1"
    ## exts."isaacsim.gui.content_browser".folders = [
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Robots",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/People",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/IsaacLab",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Props",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Environments",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Materials",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Samples",
    ##     ".../Kit/assets/Isaac/5.1/Isaac/Sensors",
    ## ]
    # open Issac Sim
    # load the assets directly from "Content" on the bottom left

To install Isaac Lab, follow <a href="https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html">installation guide</a>:

    mkdir miniconda3
    cd miniconda3/
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh -b -u -p .
    ./bin/conda init bash
    source /home/cubicdoggo/.bashrc

    conda create -n isaaclab_env python=3.11
    # update .bashrc
    # close terminal and restart
    conda activate isaaclab_env
    pip install --upgrade pip
    pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
    pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
    git clone https://github.com/isaac-sim/IsaacLab.git
    sudo apt install cmake build-essential
    mv IsaacLab/ isaaclab
    cd isaaclab
    mkdir -p /home/cubicdoggo/Documents/miniconda3/envs/isaaclab_env/lib/python3.11/site-packages/isaacsim/.vscode
    ln -sf /home/cubicdoggo/Documents/isaacsim/.vscode/settings.json /home/cubicdoggo/Documents/miniconda3/envs/isaaclab_env/lib/python3.11/site-packages/isaacsim/.vscode/settings.json
    ./isaaclab.sh --install 

And to check for the installation (may need to wait a bit):

    python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --num_envs=1000
    python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Velocity-Rough-Anymal-C-v0 --headless
    # check "Mean reward", wait 30 min
    python scripts/reinforcement_learning/rsl_rl/play.py --task=Isaac-Velocity-Rough-Anymal-C-v0
    # to find the saved trained files:
    cd /home/cubicdoggo/Documents/isaaclab/logs/rsl_rl/anymal_c_rough
    # choose a date, and exported/ should contain the corresponding policies

## References:
- AstroSam, I Made a Robot Dog (2024) (<a href="https://www.youtube.com/watch?v=XvKlplncafQ">YouTube</a>)
