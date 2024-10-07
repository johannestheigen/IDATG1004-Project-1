#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import random

ev3 = EV3Brick()

ev3.screen.print("Hello World")
ev3.speaker.beep()
#ev3.speaker.play_file('e-saying-his-name.wav')

# initialiser motorer
left_motor = Motor(Port.B)
right_motor = Motor(Port.D)

# initialiser sensorer
touch_sensor = TouchSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)


# drivebase info
robot = DriveBase(left_motor, right_motor, wheel_diameter=52, axle_track=105)


def grass_cutting_run():
    robot.drive(1000, 0)

    while True:
        # if for aa sjekke om noe er i veien
        if ultrasonic_sensor.distance() < 300:
            robot.stop()
            #ev3.speaker.play_file('error-lyd.wav')
            ev3.screen.print('noe i veien')
            
            wait(500)
            ev3.screen.clear()

            ev3.speaker.play_file('hellothere.wav')
            
            # gaa litt tilbake tilbake i ulike distanser (fra 0.5 til 1.5 sek)
            random_backwards_time = random.randint(500, 1500)
            robot.drive_time(-200, 0, random_backwards_time)
            
            # snu tilfeldig grader (ca 60-120) mot hoyre
            random_turn_angle = random.randint(170, 280)
            robot.turn(random_turn_angle)

            # fortsett aa kjor brer
            robot.drive(1000, 0)

        # hvis sensor trykkes stopp aa kjor
        if touch_sensor.pressed():
            robot.stop()
            break

        wait(100)

def main():
    while True:
        
        # start 
        if touch_sensor.pressed():

            wait(500)
            #print('starting')
            grass_cutting_run()

            ev3.screen.print("ait imma head out")
            ev3.speaker.play_file('microsoft-windows-xp-shutdown-sound.wav')
            wait(1000)
            ev3.screen.clear()
            #print('waiting for start once more')

        wait(100)

main()