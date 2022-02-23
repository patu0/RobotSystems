import sys
from click import pass_obj
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *





class Perception(object):
    def __init__(self,_target_colors):
        print("Perception Init")
        self.world_x = 0
        self.world_y = 0
        self.roi = ()
        self.size = (640, 480)
        self.rect = None
        self._target_colors = _target_colors
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            }           

    def draw_camera_lines(self, img):
        # Draw Line on incoming image
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        # Draw vertical and horizontal line on img
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        return img

    def frame_adjustments(self, img_copy):
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert images to LAB space
        return frame_lab 

    def detect_largest_contour(self, detect_color,frame_lab,color_range):
        frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # Commissioning for the original image and mask
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Lucky
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Turnover calculation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find outline
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # Find the largest contour

        return areaMaxContour, area_max 

    def insert_outline_and_text(self, img, box, detect_color,world_x,world_y, range_rgb):
        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) #绘制中心点
    
    def getAreaMaxContour(self,contours):
        # Find the largest contour area
        # Parameter is a list of contours to be compared
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # All outline
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate profile area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only the contour of the maximum area is effective when the area is greater than 300, and the interference is disturbed.
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the biggest outline
    
    def run(self,img):
        # Draw Line on incoming image
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        # Draw vertical and horizontal line on img
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_lab = self.frame_adjustments(img_copy)

        area_max = 0
        areaMaxContour = 0
        for i in color_range:
            if i in self._target_colors:
                # Detect Largest Contour of the current color
                detect_color = i
                areaMaxContour, area_max = self.detect_largest_contour(detect_color, frame_lab, color_range)
        
        if area_max > 2500:  # Check if the colored area is bigger than a certain size
            self.rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(self.rect))
            self.roi = getROI(box) #Get the ROI area
            img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, square_length)  # 获取木块中心坐标
            self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size) #转换为现实世界坐标
            
            self.insert_outline_and_text(img, box, detect_color, self.world_x, self.world_y, self.range_rgb)
        return img