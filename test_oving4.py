from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
import statistics

class TestRobot:
    def __init__(self):
        self.color_sensor = ColorSensor(Port.S1)
    
    def calibrate(self):
        black_values = []
        for _ in range(10):
            black_values.append(self.color_sensor.reflection())
        self.black_threshold = statistics.mean(black_values)
        print(self.black_threshold)

test_robot = TestRobot()
test_robot.calibrate()
