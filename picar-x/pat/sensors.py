import time, atexit
from picarx_improved import Picarx, Sensor


if __name__ == "__main__":
    px = Picarx() 
    snsr = Sensor()
    while True:
        print(snsr.chn_0.read(),snsr.chn_1.read(),snsr.chn_2.read())


