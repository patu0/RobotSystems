import time, atexit
from picarx_improved import Picarx, Sensor, Interpreter, Controller
import logging

def calibrate_sensors(snsr):
    print("CALIBRATING")
    sensor_history =[]
    num_of_calibration_iterations = 20
    for i in range(num_of_calibration_iterations):
        sensor_history.append(snsr.sensor_reading())
    sum_sensor = [0,0,0]
    for sensor_values in sensor_history:
        sum_sensor[0] += sensor_values[0]
        sum_sensor[1] += sensor_values[1]
        sum_sensor[2] += sensor_values[2]
    average_sensor_values = [sum / num_of_calibration_iterations for sum in sum_sensor]
    average_sensor_values = [int(item) for item in average_sensor_values]
    chn_sensor_value_mean = int(sum(average_sensor_values)/len(average_sensor_values))
    sensor_value_deltas = [x - chn_sensor_value_mean for x in average_sensor_values]
    print(f"Mean chn sensor value = {chn_sensor_value_mean}")
    print(f"Sensor Value Deltas= {sensor_value_deltas}")
    return sensor_value_deltas

def line_follow():
    pass


if __name__ == "__main__":
    px = Picarx() 
    snsr = Sensor()
    intptr = Interpreter()
    ctrl = Controller()
    deltas = calibrate_sensors(snsr)
    deltas_copy = deltas
    # todo turn this into a function ! 
    while True:
        print("------------------------------------------")
        cali_sensor_reading = []
        current_sensor_reading = snsr.sensor_reading()
        print("Raw Sensor Reading:",current_sensor_reading)
        deltas = deltas_copy
        print(deltas)
        zip_object = zip(deltas,current_sensor_reading)
        for deltas, current_sensor_reading in zip_object:
            cali_sensor_reading.append(current_sensor_reading-deltas)
        print("Calibrated Sensor Reading:",cali_sensor_reading)
        print(intptr.line_status(cali_sensor_reading))
        intptr.get_percentage_diff(cali_sensor_reading)
        ctrl.control(intptr.line_status(cali_sensor_reading),px)
        time.sleep(0.01)
        px.forward(0.1)
        time.sleep(0.1)
        px.stop()
        time.sleep(1)


