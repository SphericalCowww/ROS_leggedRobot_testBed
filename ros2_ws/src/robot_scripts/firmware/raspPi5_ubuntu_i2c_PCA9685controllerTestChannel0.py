# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

#######################################################################################################################
def main(args=None):
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    servo0 = servo.Servo(pca.channels[0])
    for angle in range(180):
        servo0.angle = angle
        time.sleep(0.03)
    for angle in range(180):
        servo0.angle = 180 - angle
        time.sleep(0.03)

    fraction = 0.0
    while fraction < 1.0:
        servo0.fraction = fraction
        fraction += 0.01
        time.sleep(0.03)

    pca.deinit()

#######################################################################################################################
if __name__ == "__main__": main()









