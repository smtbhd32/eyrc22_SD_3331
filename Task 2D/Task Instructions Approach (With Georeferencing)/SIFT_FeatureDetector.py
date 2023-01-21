import cv2
import numpy as np


img1 = cv2.imread("7.png",0)
img2 = cv2.imread("task2d.tif",0)

sift = cv2.SIFT_create(nfeatures = 10000)

kp1, des1 = sift.detectAndCompute(img1,None)
kp2, des2 = sift.detectAndCompute(img2,None)

# imgKp1 = cv2.drawKeypoints(img1,kp1,None)
# imgKp2 = cv2.drawKeypoints(img2,kp2,None)

bf = cv2.BFMatcher()
matches = bf.knnMatch(des1, des2, k=2)

good = []
coordinates = []

for i,(m,n) in enumerate(matches):
    if m.distance < 0.5*n.distance:
        good.append([m])
        coordinates.append(kp1[i].pt)
        print(i,kp1[i].pt)
        print(i,kp2[i].pt)
        print(...)

print(len(good))
print(len(coordinates))


img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=2)

# cv2.imshow("kp1", imgKp1)
# cv2.imshow("kp2", imgKp2)
# cv2.imshow("img1", img1)
# cv2.imshow("img2", img2)
cv2.imshow("img3", img3)
cv2.waitKey(0)