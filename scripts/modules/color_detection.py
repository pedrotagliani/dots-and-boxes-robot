import cv2
import numpy as np


image = cv2.imread('modules/test_images/1.png')

hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Lower red range
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])

# Upper red range
lower_red2 = np.array([170, 100, 100])
upper_red2 = np.array([180, 255, 255])

# lower_red1 = np.array([0, 100, 70])
# upper_red1 = np.array([10, 255, 255])
# lower_red2 = np.array([170, 100, 70])
# upper_red2 = np.array([180, 255, 255])


mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
red_mask = mask1 | mask2

red_parts = cv2.bitwise_and(image, image, mask=red_mask)

cv2.imshow('Original Image', image)
cv2.imshow('Red Parts', red_parts)

gray = cv2.cvtColor(red_parts, cv2.COLOR_BGR2GRAY)

# Apply Otsu's automatic thresholding
(T, thresh) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

cv2.imshow('thresh', thresh)

cv2.waitKey(0)
cv2.destroyAllWindows()
