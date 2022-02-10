import rossros as rr

from picarx_improved import Sensor, Controller, Interpreter, Picarx

# Variable Declaration
snsr = Sensor()
px = Picarx()
intrp = Interpreter()
cont = Controller()

# Bus Initialization
bus_sensor = rr.Bus([0,0,0],"GreyScale Sensor")
bus_line = rr.Bus(0,"Line Position")

# RossROS
p_sensor = rr.Producer(snsr.sensor_reading, bus_sensor, 0.1, name='Sense')
cp_interpretor = rr.ConsumerProducer(intrp.line_status, bus_sensor, bus_line, 0.1, name="Interpret")
c_controller = rr.Consumer(cont.control, bus_line, 0.1, name="Control")


try:
    rr.runConcurrently([])
except:
    print("Something Went Horribly Wrong!")