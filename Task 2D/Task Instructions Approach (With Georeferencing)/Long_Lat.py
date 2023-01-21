import cv2
from osgeo import gdal
import sys
#Specifying src and dest
src = "obj2.jpg"
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



#Detecting Features using Feature Matching Algo
img1 = cv2.imread(src,0)
img2 = cv2.imread(tif,0)
counter = 1.001

while counter < 2:
    print("Current_Count_" + str(counter))
    descriptor = cv2.ORB_create(nfeatures = 80000, scaleFactor = counter, nlevels = 8)

    kp1, des1 = descriptor.detectAndCompute(img1,None)
    kp2, des2 = descriptor.detectAndCompute(img2,None)


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
    gcp_list = []

    for pixel in pixels:
        x,y = pixel2coord(pixel[2],pixel[3])
        gcp = gdal.GCP(x, y, 0, pixel[0], pixel[1])
        gcp_list.append(gcp)

    finalDest = "crs_updated_" + str(counter) + ".tif"
    ds_gcp = gdal.Translate(dest, src, GCPs=gcp_list)
    ds_gcp = gdal.Warp(finalDest, ds_gcp, dstSRS='EPSG:4326')

    img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=2)
    cv2.imwrite("img3.jpg",img3)
    counter += 0.001

# cv2.imshow("kp1", imgKp1)
# cv2.imshow("kp2", imgKp2)
# cv2.imshow("img1", img1)
# cv2.imshow("img2", img2)
# cv2.imshow("img3", img3)
# cv2.waitKey(0)