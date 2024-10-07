#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# Initialize the EV3 Brick
ev3 = EV3Brick()

# Initialize motors and sensors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
touch_sensor = TouchSensor(Port.S4)
color_sensor = ColorSensor(Port.S1)

# Initialize stopwatch
stopwatch = StopWatch()

# Drive base configuration
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=160)

# Line following function
def line_follow():
    Black_Threshold = 15
    base_speed = 100
    proportional_gain = 3
    
    while True:
        # Follow the black line using the color sensor
        reflection = color_sensor.reflection()
        error = reflection - Black_Threshold
        turn_rate = proportional_gain * error

        robot.drive(base_speed, turn_rate)

        # Stop with touch sensor
        if touch_sensor.pressed():
            robot.stop()
            break

        wait(10)

# Main function to start the program
def main():
    while True:
        if touch_sensor.pressed():  # Start via touch sensor
            wait(500)
            line_follow()
            ev3.screen.print("Exiting...")
            wait(1000)
            ev3.screen.clear()

        wait(100)

# Call the main function to start the program
main()