from msilib.schema import Class
import time
# import sys
# sys.path.append(r'E:/RobotSystems/RobotSystems/picar-x/lib')
from picarx_improved import Picarx,Sensor,Interpreter,Controller
class Bus():
    def __init__(self):
        self.message = ""
    def write(self,string):
        self.message = string
    def read(self):
        return self.message


def consumer_picarx(Bus, delay_time):
    while True:
        #
        time.sleep(delay_time)

def consumer_sensor(Bus, delay_time):
    pass
def consumer_producer_interpreter(Bus, delay_time):
    while True: 
        time.sleep(delay_time)
def consumer_producer_controller(Bus, delay_time):
    while True:
        # put controller function
        #Controller.control
        time.sleep(delay_time)