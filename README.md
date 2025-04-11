# Testing for Legged Robot using ROS2

Using PCA9685 controller on DM996R servo

https://github.com/how2flow/ros2_axis6 => https://www.how2flow.net/posts/pca9685/, https://docs.circuitpython.org/projects/servokit/en/latest/
https://www.youtube.com/watch?v=-x2EEIUMm6A

    sudo apt update
    sudo apt upgrade
    sudo apt-get install raspi-config
    sudo raspi-config 	                #Navigate to Interfacing Options > I2C, and enable it.
    reboot
    sudo apt-get install -y i2c-tools python3-smbus
    i2cdetect -y 1
    sudo adduser $USER i2c
    sudo apt-get install build-essential python3 python3-dev python3-venv python3-pip
    python3 -m venv pythonEnv
    source pythonEnv/bin/activate
        pip install --upgrade pip
        pip install lgpio
        pip install adafruit-python-shell click wheel 
        pip install adafruit-circuitpython-servokit adafruit-circuitpython-pca9685 Adafruit-Blinka adafruit-circuitpython-register adafruit-circuitpython-busdevice
    deactivate


    source pythonEnv/bin/activate
    ros2 pkg create robot_interfaces
    ros2 pkg create robot_descriptions
    ros2 pkg create robot_scripts --build-type ament_python --dependencies rclpy board adafruit_motor adafruit_pca9685
    colcon build --symlink-install
    source install/setup.bash

    sudo usermod -aG i2c $USER

    ros2 run robot_scripts robot_lifecycle 
    ros2 lifecycle nodes
    ros2 lifecycle set /serial_lifecycleNode configure
    ros2 lifecycle set /serial_lifecycleNode activate
    ros2 topic list
    ros2 topic echo /serial_lifecycle_receiver

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y


## References:
- FoamyGuy, "Adafruit_CircuitPython_PCA9685" (<a href="https://github.com/adafruit/Adafruit_CircuitPython_PCA9685">GitHub</a>)
- how2flow, "ros2_axis6" (<a href="https://github.com/how2flow/ros2_axis6">GitHub</a>)
