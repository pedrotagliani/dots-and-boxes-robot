import cv2
import numpy as np

def red_color_detection(img):

    # RGB to HSV
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Lower red range
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])

    # Upper red range
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    # lower_red1 = np.array([0, 100, 70])
    # upper_red1 = np.array([10, 255, 255])
    # lower_red2 = np.array([170, 100, 70])
    # upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    red_mask = mask1 | mask2

    red_parts = cv2.bitwise_and(img, img, mask=red_mask)

    cv2.imshow('Original Image', img)
    cv2.imshow('Red Parts', red_parts)

    gray = cv2.cvtColor(red_parts, cv2.COLOR_BGR2GRAY)

    # Apply Otsu's automatic thresholding
    (T, thresh) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    cv2.imshow('thresh', thresh)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

def blue_color_detection(img):

    # RGB to HSV
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Lower blue range
    lower_blue = np.array([90, 60, 60])
    upper_blue = np.array([134, 255, 255])

    blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

    blue_parts = cv2.bitwise_and(img, img, mask=blue_mask)

    cv2.imshow('Original Image', img)
    cv2.imshow('Blue Parts', blue_parts)

    gray = cv2.cvtColor(blue_parts, cv2.COLOR_BGR2GRAY)

    # Apply Otsu's automatic thresholding
    (T, thresh) = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    cv2.imshow('thresh', thresh)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':

    # Import the image
    image = cv2.imread('modules/test_images/red_line_1.png')

    red_color_detection(image)