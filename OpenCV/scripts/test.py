import cv2
from matplotlib import pyplot as plt
import numpy as np

def cnt_area(cnt):
    area = cv2.contourArea(cnt)
    return area

frame = cv2.imread("/home/kavin/Pictures/image.png")

# print(type(frame))	# <class 'numpy.ndarray'>
# print(frame.dtype)	# uint8
print(frame.shape)	# (720, 1280, 3)
img = np.ones((720, 1280, 3), dtype=np.uint8)
# cv.namedWindow("input", cv.WINDOW_AUTOSIZE)
# cv.imshow("input", src)
# cv.waitKey(0)
# cv.destroyAllWindows()
# plt.figure(1)
# plt.imshow(frame[:, :, ::-1])

line_location = 0.5
line_location_a1 = 0.3
line_location_a2 = 0.7

h = frame.shape[0]
w = frame.shape[1]
half_h = h / 2
# print(half_h)
half_w = w / 2
# print(half_w)
l1 = int(h * (1 - line_location_a2 - 0.05))
# print(l1)
l2 = int(h * (1 - line_location_a2))
# print(l2)
line_area = frame[l1:l2, :]
# print(line_area.shape)
plt.figure(1)
plt.imshow(line_area[:, :, ::-1])

hmin, smin, vmin = 0, 0, 0
hmax, smax, vmax = 180, 255, 46
line_area = cv2.cvtColor(line_area, cv2.COLOR_BGR2HSV)
# plt.figure(2)
# plt.imshow(line_area[:, :, ::-1])
line_area = cv2.inRange(line_area, (hmin, smin, vmin), (hmax, smax, vmax))
# print(line_area.shape)
# plt.figure(2)
# plt.imshow(line_area)
kernel = np.ones((5, 5), np.uint8)
line_area = cv2.morphologyEx(line_area, cv2.MORPH_OPEN, kernel)
plt.figure(2)
plt.imshow(line_area)
image, contours, hierarchy = cv2.findContours(line_area, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours.sort(key=cnt_area, reverse=True)

# cv2.drawContours(img, contours, -1, (0, 255, 255), 3)
# plt.figure(2)
# plt.imshow(img[:, :, ::-1])

center = (0, 0)
area = -1
if len(contours) > 0:
    x, y, w, h = cv2.boundingRect(contours[0])
    cx, cy = int(x + w/2), int(y + h/2)
    area = cnt_area(contours[0])
    center = (cx, cy)

print(center)

plt.show()