# Testing for Legged Robot using ROS2

## Hardware Connections

| device | DYNAMIXEL models | number | specification |
| - | - | - | - |
| controller | <a href="https://emanual.robotis.com/docs/en/parts/controller/opencr10/">OpenCR 1.0</a> | 1 | |
| USB interface + power hub | <a href="https://emanual.robotis.com/docs/en/parts/interface/u2d2/">U2D2</a> + power hub board | ? | |
| power supply | SMPS | ? | 12V 5A | 
| servo motor | <a href="https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/">XL430-W250-T</a> | 12 | max stall torque 1.5 [N.m] (at 12.0 [V], 1.4 [A], 1.071 [Nm/A]) |

### setting the servo IDs

Connecting servo to U2D2 according to <a href="https://www.youtube.com/watch?v=FIj_NULYOKQ">YouTube</a>:

<img src="https://github.com/SphericalCowww/ROS_leggedRobot_testBed/blob/main/basicConnection_DYNAMIXEL.png" width="200">

Use the following App <a href="https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/">DYNAMIXEL Wizard 2.0</a> to change the servo ID (default 1) to, say, ID 10. Note that all servos in use from the same controller must have different IDs:

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



## References:
