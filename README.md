# Testing for Legged Robot using ROS2

## Hardware Connections

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| controller | <a href="https://emanual.robotis.com/docs/en/parts/controller/opencr10/">OpenCR 1.0</a> | 1 | This is not necessary if a Rasp Pi is already in use |
| USB interface + power hub | <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> + power hub board | 1 | Can control 12 servo in daisy chain if properly powered |
| power supply | SMPS | 4? | 12V 5A | 
| servo motor | <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | max stall torque 1.5 [N.m] (at 12.0 [V], 1.4 [A], 1.071 [Nm/A]) |

### setting the servo IDs

Connecting servo to U2D2 according to <a href="https://www.youtube.com/watch?v=FIj_NULYOKQ">YouTube</a>:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/basicConnection_DYNAMIXEL.png" width="200">

Use the following App <a href="https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/">DYNAMIXEL Wizard 2.0</a> to change the servo ID (default 1) to, say, ID 10. Note that if any of the servos have the same ID, they will NOT show up in the scan. The servos also all initially come with an ID of 1, so that must be connected one by one to U2D2 to change their IDs accordingly.

    connect U2D2 to computer => open DYNAMIXEL Wizard 2.0
    # Options 
    ## Select protocol to scan => Protocol 2.0 (only)
    ## Select port to scan => which ever port is connected
    ## Select baudrate to scan => 57600 bps (only) => OK
    # Scan
    # 7 ID => 10 ID 10 (on the right) => Save (may need to scroll down) 

Can also test out the servo:

    # LED (top right toggle)
    # Torque (top right toggle) => set the value for motion (can select Velocity/Position mode)

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
    sudo chmod a+rw /dev/ttyUSB0
    ros2 run my_toolbox_dynamixel_workbench model_scan /dev/ttyUSB0 57600

## References:
