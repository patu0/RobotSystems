import cv2
import numpy as np
import logging
import math
from picarx_improved import Picarx 
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
# frame = cv2.imread('C:/Users/97pat/Desktop/tape.png')
# # frame = cv2.imread('/home/pat/Pictures/tape_rpi.png')
# hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
# cv2.imshow("hsv", hsv)
# cv2.waitKey(0)

# Courtesy of the David Tian Towards_Data_Science
# https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96

def detect_edges(frame):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv", hsv)
    # cv2.waitKey(1000)
    lower_blue = np.array([60, 40, 40])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # cv2.imshow("blue mask", mask)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows
    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(edges):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 4 / 6),
        (width, height * 4 / 6),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    # cv2.imshow("cropped edges", cropped_edges)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows
    return cropped_edges

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=3, maxLineGap=4)
    # cv2.imshow("line segments", line_segments)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows
    # print(len(line_segments))
    return line_segments

def average_slope_intercept(frame, line_segments):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines


# def average_slope_intercept(frame, line_segments):
#     # Courtesy of the David Tian Towards_Data_Science
#     # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
#     """
#     This function combines line segments into one or two lane lines
#     If all line slopes are < 0: then we only have detected left lane
#     If all line slopes are > 0: then we only have detected right lane
#     """
#     lane_lines = []
#     if line_segments is None:
#         logging.info('No line_segment segments detected')
#         return lane_lines

#     height, width, _ = frame.shape
#     left_fit = []
#     right_fit = []

#     boundary = 0
#     left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
#     right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

#     for line_segment in line_segments:
#         for x1, y1, x2, y2 in line_segment:
#             if x1 == x2:
#                 logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
#                 continue
#             fit = np.polyfit((x1, x2), (y1, y2), 1)
#             slope = fit[0]
#             intercept = fit[1]

#             left_fit.append((slope,intercept))
#     left_fit_average = np.average(left_fit, axis=0)
#     if len(left_fit) > 0:
#         lane_lines.append(make_points(frame, left_fit_average))

#     #         if slope < 0:
#     #             if x1 < left_region_boundary and x2 < left_region_boundary:
#     #                 left_fit.append((slope, intercept))
#     #         else:
#     #             if x1 > right_region_boundary and x2 > right_region_boundary:
#     #                 right_fit.append((slope, intercept))

#     # left_fit_average = np.average(left_fit, axis=0)
#     # if len(left_fit) > 0:
#     #     lane_lines.append(make_points(frame, left_fit_average))

#     # right_fit_average = np.average(right_fit, axis=0)
#     # if len(right_fit) > 0:
#     #     lane_lines.append(make_points(frame, right_fit_average))

#     logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
#     print(lane_lines)
#     return lane_lines

def make_points(frame, line):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 4 / 6)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def detect_lane(frame):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)
    
    return lane_lines

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    # Courtesy of the David Tian Towards_Data_Science
    # https://towardsdatascience.com/deeppicar-part-4-lane-following-via-opencv-737dd9e47c96
    line_image = np.zeros_like(frame)
    if lines is not None:
        print(lines)
        for line in lines:
            print(lines)
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def picarx_angle_conversion(steering_angle_old):
    steering_angle = steering_angle_old - 90

    if steering_angle < 0:
        steering_angle = steering_angle 
    elif steering_angle > 0:
        steering_angle = steering_angle 
    else: 
        steering_angle = steering_angle
    return steering_angle

def frame_process(frame):
    # frame = cv2.imread('C:/Users/97pat/Desktop/lanes.png')
    lane_lines = detect_lane(frame)
    lane_lines_image = display_lines(frame, lane_lines)
    height, width, _ = frame.shape
    cv2.imshow("lane lines", lane_lines_image)
    # cv2.waitKey(1000)
    # cv2.destroyAllWindows
    if len(lane_lines) != 0:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

        angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
        print("steering_angle_old:",steering_angle)
        steering_angle=picarx_angle_conversion(steering_angle)
        print("steering_angle_new:", steering_angle)
        # heading_line_image=display_heading_line(frame,steering_angle)
        # cv2.imshow(heading_line_image)
        return steering_angle
    else:
        pass
# if __name__ == "__main__":
#     frame = cv2.imread('C:/Users/97pat/Desktop/lanes.png')
#     lane_lines = detect_lane(frame)
#     lane_lines_image = display_lines(frame, lane_lines)
#     height, width, _ = frame.shape
#     cv2.imshow("lane lines", lane_lines_image)
#     cv2.waitKey(1000)
#     cv2.destroyAllWindows
#     _, _, left_x2, _ = lane_lines[0][0]
#     _, _, right_x2, _ = lane_lines[1][0]
#     mid = int(width / 2)
#     x_offset = (left_x2 + right_x2) / 2 - mid
#     y_offset = int(height / 2)

#     angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
#     angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
#     steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
#     print("steering_angle:",steering_angle)

#     heading_line_image=display_heading_line(frame,steering_angle)

#     cv2.imshow("lane lines", heading_line_image)
#     cv2.waitKey(5000)
#     cv2.destroyAllWindows
#init camera
print("start color detect")
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 24
rawCapture = PiRGBArray(camera, size=camera.resolution)  
px = Picarx() 

for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):# use_video_port=True
    print("hi")
    img = frame.array
    # img,img_2,img_3 =  color_detect(img,'red')  # Color detection function
    # img =  color_detect(img,'red')  # Color detection function
    cv2.imshow("video", img)    # OpenCV image show
    
    steering_angle = frame_process(img)
    if steering_angle != None:
        px.set_dir_servo_angle(steering_angle)
        time.sleep(0.01)
        px.forward(1)
        time.sleep(0.1)
        px.stop()
    rawCapture.truncate(0)   # Release cache
   
    k = cv2.waitKey(1) & 0xFF
    # 27 is the ESC key, which means that if you press the ESC key to exit
    if k == 27:
        camera.close()
        break
