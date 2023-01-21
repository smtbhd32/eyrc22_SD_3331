import cv2
from osgeo import gdal
import sys
import numpy as np
#Specifying src and dest
src = "obj0.jpg"
dest = "georeferenced.tif"
tif = "task2d.tif"
finalDest = "crs_updated.tif"



# starting Gdal Library
ds = gdal.Open(tif) # Open tif file
# GDAL affine transform parameters, According to gdal documentation xoff/yoff are image left corner, a/e are pixel wight/height and b/d is rotation and is zero if image is north up. 
if ds is None:
    print("Could not open file!")
    sys.exit(1)
xoff, a, b, yoff, d, e = ds.GetGeoTransform()

def pixel2coord(x,y):
    """Returns global pixel from pixel x, y coords"""
    xp = a * x + b * y + xoff
    yp = d * x + e * y + yoff
    return(xp, yp)  #longitude , #latitude



#Detecting Features using Feature Matching Algo
img1 = cv2.imread(src,0)
img2 = cv2.imread(tif,0)

descriptor = cv2.SIFT_create(2000)

kp1, des1 = descriptor.detectAndCompute(img1,None)
kp2, des2 = descriptor.detectAndCompute(img2,None)


# Match the features using FLANN
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(des1,des2,k=2)

# Find the good matches
good = []
for m,n in matches:
    if m.distance < 0.7*n.distance:
        good.append(m)

# Extract the matching points
src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

# Estimate the transformation matrix using RANSAC
M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)

# Use the transformation matrix to map points from img1 to img2
dst = cv2.warpPerspective(img1, M, (img2.shape[1], img2.shape[0]))

# Define the pixel of interest in the first image
sample_pixel = [320,240]

# Use the transformation matrix to map the sample pixel to the second image
transformed_pixel = cv2.perspectiveTransform(np.array([[sample_pixel]], dtype=np.float32), M)

# The transformed_pixel is a 2x1x2 array, so we can extract the x and y coordinates
# of the transformed pixel in the second image
x = int(transformed_pixel[0][0][0])
y = int(transformed_pixel[0][0][1])

# Print the location of the transformed pixel in the second image
print("Transformed pixel location in second image: ({}, {})".format(x, y))

corresponding_pixel_value = img2[y][x]

# Print the pixel value of the transformed pixel in the second image
print("Pixel value at transformed pixel location: {}".format(corresponding_pixel_value))

# Draw circle on the transformed pixel in the second image
cv2.circle(img2, (x, y), 10, (0, 0, 255), 15)

# write the image with the circle drawn on it
cv2.imwrite("transformed_pixel.png", img2)

# Get longitude and latitude of the transformed pixel
lon, lat = pixel2coord(x, y)

# Print the longitude and latitude of the transformed pixel
print("Longitude: {}, Latitude: {}".format(lon, lat))

