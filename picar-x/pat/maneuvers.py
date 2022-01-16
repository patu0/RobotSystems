import time, atexit
from picarx_improved import Picarx 

def forward_and_backward(maneuver_num,calib_ang):
    print(f"You picked {maneuver_num} | Move Forward and Backward")
    px.set_dir_servo_angle(calib_ang)
    px.forward(10)
    time.sleep(1)
    px.forward(0)
    time.sleep(1)
    px.backward(10)
    time.sleep(1)
    px.backward(0)
    time.sleep(1)
    px.stop()
def parallel_park_left(maneuver_num,calib_ang):
    print(f"You picked {maneuver_num} | Parallel Park Left")
    px.set_dir_servo_angle(calib_ang-25)
    time.sleep(0.01)
    px.backward(2)
    time.sleep(1)
    px.set_dir_servo_angle(calib_ang+20)
    time.sleep(0.01)
    px.backward(1)
    time.sleep(1)
    px.set_dir_servo_angle(3)
    time.sleep(0.01)
    px.backward(0)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(calib_ang)
def parallel_park_right(maneuver_num,calib_ang):
    print(f"You picked {maneuver_num} | Parallel Park Right")
    px.set_dir_servo_angle(calib_ang+25)
    time.sleep(0.01)
    px.backward(2)
    time.sleep(1)
    px.set_dir_servo_angle(calib_ang-20)
    time.sleep(0.01)
    px.backward(1)
    time.sleep(1)
    px.set_dir_servo_angle(3)
    time.sleep(0.01)
    px.backward(0)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(calib_ang)
def three_point_turn(maneuver_num,calib_ang):
    print(f"You picked {maneuver_num} | Three-Point Turn")
    px.set_dir_servo_angle(calib_ang-40)
    time.sleep(0.01)
    px.forward(3)
    time.sleep(2)
    px.stop()
    px.set_dir_servo_angle(calib_ang+40)
    time.sleep(0.01)
    px.backward(2)
    time.sleep(1)
    px.stop()
    px.set_dir_servo_angle(calib_ang-20)
    time.sleep(0.01)
    px.forward(3)
    time.sleep(1.2)
    px.set_dir_servo_angle(calib_ang)
    time.sleep(0.01)
    px.forward(3)
    time.sleep(0.5)
    px.stop()


if __name__ == "__main__":
    px = Picarx()
    # px.dir_servo_angle_calibration(3)
    calib_ang = 3

    while True:
        maneuver_num = input ("What Maneuver would you like the robot to perform? \n 1: Move Forward and Backward \n 2: Parallel Park to the Left \n 3: Parallel Park to the Right \n 4: Three Point Turn \n 5: Exit Selection \n")

        if maneuver_num == "1":
            forward_and_backward(maneuver_num,calib_ang)
        elif maneuver_num == "2":
            parallel_park_left(maneuver_num,calib_ang)
        elif maneuver_num == "3":
            parallel_park_right(maneuver_num,calib_ang)
        elif maneuver_num == "4":
            three_point_turn(maneuver_num,calib_ang)
        elif maneuver_num == "5":
            break
        else:
            print(f"You picked {maneuver_num} | Please pick a valid number!!")
    atexit.register(px.stop)