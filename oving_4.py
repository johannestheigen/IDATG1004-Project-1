#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import statistics

class LineFollowerRobot:
    def __init__(self):
        """
        Initializes the EV3 Line Follower Robot with motors, sensors, and a drive base.
        """
        # Initialize EV3 brick and motors
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.D)
        self.right_motor = Motor(Port.A)
        self.touch_sensor = TouchSensor(Port.S4)
        self.color_sensor = ColorSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S3)

        # Initialize drive base with motors
        self.robot = DriveBase(self.left_motor, self.right_motor, wheel_diameter=56, axle_track=160)

        # Initial values for black and white thresholds
        self.black_threshold = None
        self.white_threshold = None
        self.lower_reflection_threshold = None
        self.upper_reflection_threshold = None

        # Initial gain and speed values
        self.proportional_gain = 2.5
        self.derivative_gain = 1.0
        self.base_speed = 150

        # Store the previous error for derivative control
        self.previous_error = 0

    def calibrate_sensors(self):
        """
        Calibrate the color sensor to determine the black and white thresholds.
        """
        self.ev3.screen.print("Calibration: Place on black, press touch sensor.")
        while not self.touch_sensor.pressed():
            wait(10)  # Wait until the touch sensor is pressed

        # Take multiple samples of the black line reflection
        black_values = [self.color_sensor.reflection() for _ in range(10)]
        self.black_threshold = statistics.mean(black_values)
        self.ev3.screen.print(f"Black threshold: {self.black_threshold}")
        wait(1000)

        self.ev3.screen.print("Place on white, press touch sensor.")
        while not self.touch_sensor.pressed():
            wait(10)

        # Multiple white samples
        white_values = [self.color_sensor.reflection() for _ in range(10)]
        self.white_threshold = statistics.mean(white_values)
        self.ev3.screen.print(f"White threshold: {self.white_threshold}")
        wait(1000)

        # Define the lower and upper reflection range based on the calibrated values
        self.lower_reflection_threshold = self.black_threshold - 5
        self.upper_reflection_threshold = self.white_threshold + 5

        # Calculate the target reflection value as the midpoint
        self.target_reflection = (self.black_threshold + self.white_threshold) / 2
        self.ev3.screen.print(f"Target reflection: {self.target_reflection}")
        wait(1000)

    def follow_line(self):
        """
        Follows the line using the color sensor and maintains a straight path with the gyro sensor.
        """
        # Reset the gyro sensor angle at the start
        self.gyro_sensor.reset_angle(0)
        target_angle = 0

        while True:
            # Get the current reflection and calculate error
            reflection = self.color_sensor.reflection()

            # Determine if the reflection value falls within range
            if reflection < self.lower_reflection_threshold or reflection > self.upper_reflection_threshold:
                # If out of range, adjust robot's position aggressively to find the line
                self.ev3.screen.print(f"Out of range! Reflection: {reflection}")
                self.robot.drive(0, 100)  # Turn in place to find the line
                continue  # Skip to the next loop iteration

            error = reflection - self.target_reflection

            # Calculate derivative (change in error)
            derivative = error - self.previous_error

            # Proportional-Derivative (PD) control
            turn_rate = (self.proportional_gain * error) + (self.derivative_gain * derivative)

            # Get current gyro angle and apply correction
            current_angle = self.gyro_sensor.angle()
            angle_error = current_angle - target_angle

            # Gyro correction factor to minimize drift
            gyro_correction = angle_error * 2  # Adjust through trial and error

            # Drive the robot with the calculated turn rate and gyro correction
            self.robot.drive(self.base_speed, turn_rate - gyro_correction)

            # Update previous error for the next loop
            self.previous_error = error

            # Stop the robot when the touch sensor is pressed
            if self.touch_sensor.pressed():
                self.robot.stop(Stop.BRAKE)
                break

            wait(10)

    def run(self):
        """
        Waits for the touch sensor to start or stop the robot, and executes calibration.
        """
        # Calibrate the sensors before starting
        self.calibrate_sensors()

        self.ev3.screen.print("Waiting for touch to start line following.")
        while True:
            # Wait for the touch sensor to start the line following routine
            if self.touch_sensor.pressed():
                wait(500)  
                self.follow_line()
                self.ev3.screen.print("Line following stopped. Waiting for restart...")
                wait(1000) 
                self.ev3.screen.clear()

            wait(100)  


robot = LineFollowerRobot()
robot.run()
