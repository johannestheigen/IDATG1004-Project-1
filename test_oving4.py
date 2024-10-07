#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

class RallyRobot():
    def __init__(self):
        """
        Initialize the RallyRobot with a DriveBase, 1 color sensor, 1 gyro sensor, and 1 touch sensor.
        """
        # Initialize EV3 brick and motors
        self.ev3 = EV3Brick()
        left_motor = Motor(Port.D)
        right_motor = Motor(Port.A)

        # Initialize the drive base with left and right motors
        self.robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=160)

        # Initialize sensors
        self.color_sensor = ColorSensor(Port.S1)
        self.gyro_sensor = GyroSensor(Port.S3)
        self.touch_sensor = TouchSensor(Port.S4)

        # Set initial target angle for gyro (used for straight-line correction)
        self.gyro_sensor.reset_angle(0)
        self.target_angle = 0

        # Default thresholds for line detection
        self.black_threshold = 15  # Default value; adjust this after calibration
        self.white_threshold = 70  # Default value; adjust this after calibration

        # State variable to track whether the robot is currently driving
        self.driving = False  # Start with the robot stopped

        # Proportional gain for steering control
        self.proportional_gain = 1.5  # Start with a lower gain value

    def calibrate_sensors(self):
        """
        Calibrates the color sensor by measuring the black and white reflection values.
        """
        self.ev3.screen.print("touch sensor to start calibration")
        print("Starting calibration. Place the sensor over the black line and then the white background.")
        print("Press the touch sensor to read the values.")
        
        # Measure black line reflection
        print("Place the sensor over the black line and press the touch sensor.")
        while not self.touch_sensor.pressed():
            wait(10)
        black_value = self.color_sensor.reflection()
        print("Measured black line reflection value: {}".format(black_value))
        self.ev3.screen.print(black_value, "black")
        wait(1000)  # Wait for user to move sensor

        # Measure white background reflection
        print("Now, place the sensor over the white background and press the touch sensor.")
        while not self.touch_sensor.pressed():
            wait(10)
        white_value = self.color_sensor.reflection()
        print("Measured white background reflection value: {}".format(white_value))
        self.ev3.screen.print(white_value, "white")
        self.ev3.screen.print("touch to start")

        # Update thresholds based on measured values
        self.black_threshold = black_value
        self.white_threshold = white_value
        self.target_reflection = (self.black_threshold + self.white_threshold) / 2

        print("Calibration complete. Black Threshold: {}, White Threshold: {}, Target Reflection: {}"
              .format(self.black_threshold, self.white_threshold, self.target_reflection))

        # Wait for the touch sensor to be released before proceeding
        while self.touch_sensor.pressed():
            wait(10)

    def follow_line(self):
        """
        Follows the rally-path by using the color sensor.
        """
        # Read the current reflection value from the color sensor
        reflection = self.color_sensor.reflection()

        # Calculate the error based on the target reflection value
        error = reflection - self.target_reflection

        # Print reflection and error values for debugging
        print("Reflection: {}, Error: {}, Target: {}".format(reflection, error, self.target_reflection))

        # Calculate turn rate based on the error
        turn_rate = self.proportional_gain * error

        # Disable gyro correction temporarily to simplify behavior
        correction = 0

        # Drive the robot with base speed and turn rate (without gyro correction)
        base_speed = 150  # Use a lower base speed to observe behavior more clearly
        self.robot.drive(base_speed, turn_rate - correction)

    def toggle_driving(self):
        """
        Toggles the driving state of the robot between driving and stopped.
        """
        # Toggle the driving state
        self.driving = not self.driving

        if self.driving:
            print("Starting to drive.")
            self.ev3.speaker.beep()  # Play a sound to indicate starting
        else:
            # If driving is False, stop the robot
            self.robot.stop(Stop.BRAKE)
            print("Robot is stopped.")
            self.ev3.speaker.beep()  # Play a sound to indicate stopping

    def run(self):
        """
        Waits for the touch sensor to start or stop the robot.
        """
        # Run the calibration routine before starting
        self.calibrate_sensors()

        print("Waiting for touch sensor press to start driving.")
        while True:
            # Wait for the touch sensor to be pressed
            if self.touch_sensor.pressed():
                wait(200)  # Debounce delay to avoid multiple toggles
                self.toggle_driving()  # Toggle the driving state

                # Wait for the touch sensor to be released before proceeding
                while self.touch_sensor.pressed():
                    wait(10)  # Short delay to wait for release

            # If driving is enabled, follow the line
            if self.driving:
                self.follow_line()

# Instantiate an object of the RallyRobot class
rally_robot = RallyRobot()

# Call the ‘run’ method to start the robot control logic
rally_robot.run()