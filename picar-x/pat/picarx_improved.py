import time
try:
    import sys
    sys.path.append(r'/home/pat/Documents/RobotSystems/picar-x/lib')
    from utils import *
    from utils import reset_mcu
    from servo import Servo 
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    reset_mcu()
    time.sleep (0.01)
    print("This is the PiCar-X System")
except ImportError:
    print ("This computer does not appear to be a PiCar -X system (ezblock is not present). Shadowing hardware calls with substitute functions ")
    from sim_ezblock import *



class Picarx(object):
    PERIOD = 4095
    PRESCALER = 1 # originally 10 
    TIMEOUT = 0.02

    def __init__(self):
        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.config_flie = fileDB('/home/pat/.config')
        self.dir_cal_value = int("0")
        self.cam_cal_value_1 = int("0")
        self.cam_cal_value_2 = int("0")
        # self.dir_cal_value = int(self.config_flie.get("picarx_dir_servo", default_value=0))
        # self.cam_cal_value_1 = int(self.config_flie.get("picarx_cam1_servo", default_value=0))
        # self.cam_cal_value_2 = int(self.config_flie.get("picarx_cam2_servo", default_value=0))
        self.dir_servo_pin.angle(self.dir_cal_value)
        self.camera_servo_pin1.angle(self.cam_cal_value_1)
        self.camera_servo_pin2.angle(self.cam_cal_value_2)

        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")


        self.S0 = ADC('A0')
        self.S1 = ADC('A1')
        self.S2 = ADC('A2')

        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1,1]")
        self.cali_dir_value = "[1,1]"
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

    def set_motor_speed(self,motor,speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self,value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self,motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)


    def dir_servo_angle_calibration(self,value):
        self.dir_cal_value = value
        print("calibrationdir_cal_value:",self.dir_cal_value)
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self,value):
        self.dir_current_angle = value
        angle_value  = value + self.dir_cal_value
        print("angle_value:",angle_value)
        # print("set_dir_servo_angle_1:",angle_value)
        # print("set_dir_servo_angle_2:",dir_cal_value)
        self.dir_servo_pin.angle(angle_value)

    def camera_servo1_angle_calibration(self,value):
        self.cam_cal_value_1 = value
        self.config_flie.set("picarx_cam1_servo", "%s"%value)
        print("cam_cal_value_1:",self.cam_cal_value_1)
        self.camera_servo_pin1.angle(value)

    def camera_servo2_angle_calibration(self,value):
        self.cam_cal_value_2 = value
        self.config_flie.set("picarx_cam2_servo", "%s"%value)
        print("picarx_cam2_servo:",self.cam_cal_value_2)
        self.camera_servo_pin2.angle(value)

    def set_camera_servo1_angle(self,value):
        self.camera_servo_pin1.angle(-1*(value + -1*self.cam_cal_value_1))
        # print("self.cam_cal_value_1:",self.cam_cal_value_1)
        print((value + self.cam_cal_value_1))

    def set_camera_servo2_angle(self,value):
        self.camera_servo_pin2.angle(-1*(value + -1*self.cam_cal_value_2))
        # print("self.cam_cal_value_2:",self.cam_cal_value_2)
        print((value + self.cam_cal_value_2))

    def get_adc_value(self):
        adc_value_list = []
        adc_value_list.append(self.S0.read())
        adc_value_list.append(self.S1.read())
        adc_value_list.append(self.S2.read())
        return adc_value_list

    def set_power(self,speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed) 

    def backward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0 
            print("power_scale:",power_scale)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
            else:
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
        else:
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self,speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            # if abs_current_angle >= 0:
            if abs_current_angle > 40:
                abs_current_angle = 40
            power_scale = (100 - abs_current_angle) / 100.0 
            print("power_scale:",power_scale)
            if (current_angle / abs_current_angle) > 0:
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
            else:
                self.set_motor_speed(1, speed * power_scale)
                self.set_motor_speed(2, -1*speed )
        else:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def cleanup(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
        self.set_dir_servo_angle(0)

class Sensor(object):
    def __init__(self):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.reference = 250

    def sensor_reading(self):      
        list_of_outputs = [self.chn_0.read(),self.chn_1.read(),self.chn_2.read()]
        return list_of_outputs
    def cali_sensor_reading():
        pass
class Interpreter(object):
    def __init__(self):
        pass
    def line_status(self,sensor_value_list,reference):
        pertcentage_diffs = self.get_percentage_diff(sensor_value_list)


        print(sensor_value_list, reference)
        if sensor_value_list[0] < reference and sensor_value_list[1] < reference and sensor_value_list[2] < reference: 
            return 'STOP'
        elif sensor_value_list[1] >= reference: 
            return 'FORWARD'
        elif sensor_value_list[0] >= reference: 
            return 'LEFT', 1
        elif sensor_value_list[2] >= reference: 
            return 'RIGHT' , -1
        else:
            return 'Somethings Wrong.... :^)'
    def get_percentage_diff(self, sensor_values):
        percent_diff_0_and_1 = sensor_values[1]/sensor_values[0]
        percent_diff_1_and_2 = sensor_values[1]/sensor_values[2]
        print(percent_diff_0_and_1,percent_diff_1_and_2)
        return percent_diff_0_and_1,percent_diff_1_and_2
class Controller(object):
    def __init__(self, scaling_factor=100):
        pass
