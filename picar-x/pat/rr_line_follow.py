import rossros as rr

from picarx_improved import Sensor, Controller, Interpreter, Picarx

sensor = Sensor()
car = Picarx()



p_sensor = rr.Producer()

cp_interpretor = rr.ConsumerProducer()

c_controller = rr.Consumer()

rr.runConcurrently([])