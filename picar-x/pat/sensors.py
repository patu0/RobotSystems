import time, atexit
from picarx_improved import Picarx, Sensor


if __name__ == "__main__":
    px = Picarx() 
    snsr = Sensor()
    print(snsr.chn_0.read())