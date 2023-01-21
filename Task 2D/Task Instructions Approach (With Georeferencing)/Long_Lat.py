import cv2
import numpy as np
from osgeo import gdal
import subprocess
import shlex
import sys

#Specifying src and dest
src = "obj2.jpg"
src1 = "img1.jpg"
dest = "georeferenced.tif"
tif = "task2d.tif"
finalDest = "crs_updated.tif"



#starting Gdal Library
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



#Detecting Features using SIFT Algo
img1 = cv2.imread(src,0)
img2 = cv2.imread(tif,0)

# 1.9119999999998996, 1.5159999999999432
sift = cv2.ORB_create(nfeatures = 60000, scaleFactor = 1.9119999999998996, nlevels = 8)

kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# imgKp1 = cv2.drawKeypoints(img1,kp1,None)
# imgKp2 = cv2.drawKeypoints(img2,kp2,None)

bf = cv2.BFMatcher()
matches = bf.knnMatch(des1, des2, k=2)

good = []
pixels = []

for i,(m,n) in enumerate(matches):
    if m.distance < 0.50*n.distance:
        good.append([m])
        pixels.append(kp1[i].pt)
        pixels[len(pixels) - 1] = pixels[len(pixels) - 1] + kp2[i].pt
        # print(i,kp1[i].pt)
        # print(i,kp2[i].pt)
        # print(pixels[len(pixels) - 1])
print(len(good))


#Georeferencing using Subprocess Module and GDAL
command = "gdal_translate"
i = 0
for pixel in pixels:
    x,y = pixel2coord(pixel[2],pixel[3])
    # coord[len(coord) - 1] += pixel2coord(pixel[2],pixel[3])
    command = command + " -gcp " + str(pixel[0]) + " " + str(pixel[1]) + " " + str(x) + " " + str(y)
    i += 1
    if i > 30:
        break

command = command + " -of GTiff " + src + " " + dest
print(command)

# gdal_translate -of GTiff -gcp 62.6434 361.496 -122.156 37.4359 -gcp 404.988 321.995 -122.155 37.436 -gcp 515.112 227.431 -122.155 37.4361 -gcp 438.504 356.708 -122.155 37.4359 -gcp 81.7955 171.172 -122.156 37.4363 -gcp 65.0374 227.431 -122.156 37.4362 -gcp 154.813 296.858 -122.156 37.436 -gcp 403.791 118.504 -122.155 37.4363 -gcp 387.032 190.324 -122.155 37.4362 -gcp 246.983 45.4863 -122.155 37.4365 "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/Task 2D/drone.png" "/tmp/drone.png"
# gdalwarp -r near -order 3 -co COMPRESS=NONE  -t_srs EPSG:4326 "/tmp/drone.png" "/home/missmessedup/catkin_ws/src/sentinel_drone/sentinel_drone/scripts/Task 2D/drone_modified.tif"
subprocess.run(shlex.split(command))



#Correcting the Coordinate System
from_SRS = "EPSG:4326"
to_SRS = "EPSG:4326"

# cmd_list = "gdalwarp -r near -order 2 -co COMPRESS=NONE -t_srs " + to_SRS + " -overwrite " + dest + " " + finalDest
cmd_list = "gdalwarp -r bilinear -s_srs " + from_SRS + " -t_srs "+ to_SRS + " -overwrite " + dest + " " + finalDest
print(cmd_list)
subprocess.run(shlex.split(cmd_list))



img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=2)
cv2.imwrite("img3.jpg",img3)

# cv2.imshow("kp1", imgKp1)
# cv2.imshow("kp2", imgKp2)
# cv2.imshow("img1", img1)
# cv2.imshow("img2", img2)
# cv2.imshow("img3", img3)
# cv2.waitKey(0)