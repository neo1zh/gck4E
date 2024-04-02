import cv2 as cv
import numpy as np

# Load the image
img = cv.imread('./fig2points/cxk.png', cv.IMREAD_GRAYSCALE)

# Threshold the image to create a binary image
_, binary_img = cv.threshold(img, 128, 255, cv.THRESH_BINARY)

# Find contours
contours, _ = cv.findContours(binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

# Create an empty image for drawing contours
contour_img = np.zeros_like(img)

# Draw contours
cv.drawContours(contour_img, contours, -1, (255), thickness=cv.FILLED)

# Display the result
cv.imshow('Original Image', img)
cv.imshow('Contours', contour_img)
cv.waitKey(0)
cv.destroyAllWindows()
