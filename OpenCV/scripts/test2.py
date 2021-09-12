#!/usr/bin/enc python

import cv2
from matplotlib import pyplot as plt
import numpy as np
import math
from geographic_msgs.msg import Pose, Point, Quaternion

line_location = 0.5
line_location_a1 = 0.3
line_location_a2 = 0.7
line_color = 'black'

def cnt_area(cnt):
    area = cv2.contourArea(cnt)
    return area

def get_line_area(frame):
    global line_location, line_location_a1, line_location_a2, line_color
    global cy_a1, cy_a2, half_h, half_w

    h = frame.shape[0]
    half_h = h / 2
    half_w = frame.shape[1] / 2
    l1 = int(h * (1 - line_location - 0.05))
    l2 = int(h * (1 - line_location))
    line_area = frame[l1:l2, :]

    l1 = int(h * (1 - line_location_a1 - 0.05))
    l2 = int(h * (1 - line_location_a1))
    line_area_a1 = frame[l1:l2, :]
    cy_a1 = l1

    l1 = int(h * (1 - line_location_a2 - 0.05))
    l2 = int(h * (1 - line_location_a2))
    cy_a2 = l1
    line_area_a2 = frame[l1:l2, :]

    return line_area, line_area_a1, line_area_a2

def seg(line_area, line_area_a1, line_area_a2, _line_color='black'):
    if _line_color == 'black':
        hmin, smin, vmin = 0, 0, 0
        hmax, smax, vmax = 180, 255, 46

    line_area = cv2.cvtColor(line_area, cv2.COLOR_BGR2HSV)
    line_area = cv2.inRange(line_area, (hmin, smin, vmin), (hmax, smax, vmax))

    kernel = np.ones((5, 5), np.uint8)
    line_area = cv2.morphologyEx(line_area, cv2.MORPH_OPEN, kernel)

    kernel = np.ones((5, 5), np.uint8)
    line_area = cv2.morphologyEx(line_area, cv2.MORPH_CLOSE, kernel)

    image, contours, hierarchy = cv2.findContours(line_area, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=cnt_area, reverse=True)

    center = (0, 0)
    area = -1
    if len(contours) > 0:
        x, y, w, h = cv2.boundingRect(contours[0])
        cx, cy = int(x + w/2), int(y + h/2)
        area = cnt_area(contours[0])
        center = (cx, cy)
    
    line_area_a1 = cv2.cvtColor(line_area_a1, cv2.COLOR_BGR2HSV)
    line_area_a1 = cv2.inRange(line_area_a1, (hmin, smin, vmin), (hmax, smax, vmax))

    kernel = np.ones((5, 5), np.uint8)
    line_area_a1 = cv2.morphologyEx(line_area_a1, cv2.MORPH_OPEN, kernel)

    kernel = np.ones((5, 5), np.uint8)
    line_area_a1 = cv2.morphologyEx(line_area_a1, cv2.MORPH_CLOSE, kernel)

    image, contours_a1, hierarchy = cv2.findContours(line_area_a1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_a1.sort(key=cnt_area, reverse=True)

    global cy_a1, cy_a2, half_h, half_w
    center_a1 = (0, 0)
    if len(contours_a1) > 0:
        x, y, w, h = cv2.boundingRect(contours_a1[0])
        cx, cy = int(x + w/2), int(y + h/2) + cy_a1
        center_a1 = (cx - half_w, cy - half_h)

    line_area_a2 = cv2.cvtColor(line_area_a2, cv2.COLOR_BGR2HSV)
    line_area_a2 = cv2.inRange(line_area_a2, (hmin, smin, vmin), (hmax, smax, vmax))

    kernel = np.ones((5, 5), np.uint8)
    line_area_a2 = cv2.morphologyEx(line_area_a2, cv2.MORPH_OPEN, kernel)

    kernel = np.ones((5, 5), np.uint8)
    line_area_a2 = cv2.morphologyEx(line_area_a2, cv2.MORPH_CLOSE, kernel)

    image, contours_a2, hierarchy = cv2.findContours(line_area_a2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_a2.sort(key=cnt_area, reverse=True)

    center_a2 = (0, 0)
    if len(contours_a2) > 0:
        x, y, w, h = cv2.boundingRect(contours_a2[0])
        cx, cy = int(x + w/2), int(y + h/2) + cy_a2
        center_a2 = (cx - half_w, cy - half_h)

    return line_area, center, area, center_a1, center_a2

def main():
    camera_matrix = [[369.50208, 0, 640]
                    [0, 369.50208, 360]
                    [0, 0, 1]]

    frame = cv2.imread("/home/kavin/Pictures/image2.png")
    area_base, area_base_a1, area_base_a2 = get_line_area(frame)
    # plt.figure(1)
    # plt.imshow(area_base[:, :, ::-1])
    # plt.figure(2)
    # plt.imshow(area_base_a1[:, :, ::-1])
    # plt.figure(3)
    # plt.imshow(area_base_a2[:, :, ::-1])

    area, cxcy, a, center_a1, center_a2 = seg(area_base, area_base_a1, area_base_a2, _line_color=line_color)
    # plt.figure(1)
    # plt.imshow(area)
    print(cxcy)
    print(a)

    # if a > 0:
    cv2.circle(area, (cxcy[0], cxcy[1]), 4, (0, 0, 255), -1)
    angle = (cxcy[0] - camera_matrix[0,2]) / camera_matrix[0,2] * math.atan((area.shape[1] / 2) / camera_matrix[0,0])
    pose = Pose(Point(angle, 1, 0), Quaternion(center_a1[0], center_a1[1], center_a2[0], center_a2[1]))
    print(pose)
    plt.show()

if __name__ == '__main__':
    main()