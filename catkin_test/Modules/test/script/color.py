#! /usr/bin/env python
# -*- coding: utf-8 -*-
import cv2 as cv

src = cv.imread("/home/kavin/Pictures/test.png")
print(cv.__version__)
print(type(src))	# <class 'numpy.ndarray'>
print(src.dtype)	# uint8
print(src.shape)	# (610, 570, 3)
cv.namedWindow("input", cv.WINDOW_AUTOSIZE)
cv.imshow("input", src)
cv.waitKey(0)
cv.destroyAllWindows()
