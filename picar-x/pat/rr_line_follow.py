import rossros as rr

from picarx_improved import Sensor, Controller, Interpreter, Picarx

sensor = Sensor()
car = Picarx()
controller

p_sensor = rr.Producer(Sensor.sensor_reading, sensor_bus, 0.1, name='Sensor')

cp_interpretor = rr.ConsumerProducer(Interpreter.)

c_controller = rr.Consumer()

try:
    rr.runConcurrently([])
except:
    print("Something Went Horribly Wrong!")