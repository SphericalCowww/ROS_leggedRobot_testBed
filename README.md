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


## References:
- FoamyGuy, "Adafruit_CircuitPython_PCA9685" (<a href="https://github.com/adafruit/Adafruit_CircuitPython_PCA9685">GitHub</a>)
- how2flow, "ros2_axis6" (<a href="https://github.com/how2flow/ros2_axis6">GitHub</a>)
