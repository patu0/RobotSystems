import time, atexit
from picarx_improved import Picarx, Sensor
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
    chn_sensor_value_mean = sum(average_sensor_values)/len(average_sensor_values)
    sensor_value_deltas = [x - chn_sensor_value_mean for x in average_sensor_values]
    print(f"Mean chn sensor value = {chn_sensor_value_mean}")
    print(f"Sensor Value Deltas= {sensor_value_deltas}")
    return sensor_value_deltas


if __name__ == "__main__":
    px = Picarx() 
    snsr = Sensor()
    calibrate_sensors(snsr)
    while True:
        print(snsr.chn_0.read(),snsr.chn_1.read(),snsr.chn_2.read())
        time.sleep(1)


