import time
from picarx_improved import Picarx,Sensor,Interpreter,Controller
import concurrent.futures
from picamera.array import PiRGBArray
from picamera import PiCamera
from lane_follow import frame_process
import sys
# sys.path.append(r'E:\\Code_Projects\\RobotSystems\\Lib\\site-packages')
import cv2
# import sys
# sys.path.append(r'E:/RobotSystems/RobotSystems/picar-x/lib')


class Bus():
    def __init__(self, message ):
        self.message = message
    def write(self,message):
        self.message = message
        return message
    def read(self):
        return self.message

def producer_camera(sensor_bus,delay_time):
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 24
    rawCapture = PiRGBArray(camera, size=camera.resolution) 

    frame = camera.capture(rawCapture, format="bgr",use_video_port=True)
    img = frame.array
    cv2.imshow("capture", img)
    rawCapture.truncate(0)
    sensor_bus.write(img)

def consumer_producer_interpreter(sensor_bus, interpreter_bus, delay_time):
    while True:
        img = sensor_bus.read()

        steering_angle = frame_process(img)
        interpreter_bus.write(steering_angle)

        time.sleep(delay_time)
def consumer_controller(interpreter_bus, delay_time):
    # Read Interpreter Bus, update Picarx Angle
    px = Picarx()
    steering_angle = interpreter_bus.read()
    px.set_dir_servo_angle(steering_angle)



if __name__ == '__main__':
    sensor_function = producer_camera
    sensor_values_bus = Bus(0)
    sensor_delay = 1

    interpreter_function = consumer_producer_interpreter
    interpreter_bus = Bus(0)
    interpreter_delay = 1

    controller_function = consumer_controller
    controller_delay = 1

    with concurrent.futures.ThreadPoolExecutor(max_worker=2) as executor:
        eSensor = executor.submit(sensor_function, sensor_values_bus,sensor_delay)
        eInterpreter = executor.submit(interpreter_function, sensor_values_bus, interpreter_bus, interpreter_delay)
        eController = executor.submit(controller_function, interpreter_bus, controller_delay)
    eSensor.result()