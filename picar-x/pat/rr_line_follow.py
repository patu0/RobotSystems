import rossros as rr
from picarx_improved import Picarx, Sensor, Controller, Interpreter, Sensor_Ultrasonic, Interpreter_Ultrasonic, Controller_Ultrasonic

snsr = Sensor()
px = Picarx()
intrp = Interpreter()
cont = Controller()
snsr_ultra = Sensor_Ultrasonic()
intrp_ultra = Interpreter_Ultrasonic()
cont_ultra = Controller_Ultrasonic()

# Line Tracking Bus Initialization
bus_sensor = rr.Bus([0,0,0],"GreyScale Sensor")
bus_line = rr.Bus(0,"Line Position")
# UltraSonic Sensing Bus Initialization
bus_ultrasonic_sensor = rr.Bus(0, 'Ultrasonic Sensor')
bus_too_close = rr.Bus(False, "Too Close") 
# Other Bus Initialization!
bus_termination = rr.Bus()

# RossROS Implementation Greyscale Ground Sensor
p_sensor = rr.Producer(snsr.sensor_reading, bus_sensor, 0.1, name='Sense')
cp_interpretor = rr.ConsumerProducer(intrp.line_status, bus_sensor, bus_line, 0.1, name="Interpret")
c_controller = rr.Consumer(cont.control, bus_line, 0.1, name="Control")
# RossROS Implementation Ultrasonic Sensor
p_ultrasonic_sensor = rr.Producer(snsr_ultra.read, bus_ultrasonic_sensor, 0.1, name='Ultrasonic Sense')
cp_ultrasonic_interpreter = rr.ConsumerProducer(intrp_ultra.process, bus_ultrasonic_sensor,bus_too_close,0.1, name='Ultrasonic Interpret')
c_ultrasonic_controller = rr.Consumer(cont_ultra.emergency_stop,bus_too_close,0.1, name='Emergency Stop')

p_timer = rr.Timer(rr.default_termination_bus)

try:
    rr.runConcurrently([p_sensor,cp_interpretor,c_controller,
                        p_ultrasonic_sensor, cp_ultrasonic_interpreter, c_ultrasonic_controller,
                        p_timer])
except:
    print("Oh no Something Went Horribly Wrong! :,(")