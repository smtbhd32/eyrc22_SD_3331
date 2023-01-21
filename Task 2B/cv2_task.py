import numpy as np
import cv2

img = cv2.imread("5.png", -1)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_ylow = np.array([20, 130, 130])
upper_ylow = np.array([30, 255, 255])

mask = cv2.inRange(hsv, lower_ylow, upper_ylow)

corners = cv2.goodFeaturesToTrack(mask, 200, 0.001 , 5)
corners = np.int0(corners)

detectedx = []
detectedy = []
for corner in corners:
    x, y = corner.ravel()
    detectedx.append(x)
    detectedy.append(y)
    
x = int(sum(detectedx)/len(detectedx))
y = int(sum(detectedy)/len(detectedy))
cv2.circle(img, (x,y), 5, (255, 0, 0), -1)

Area = (max(detectedx)-min(detectedx))*(max(detectedy)-min(detectedy))
print(Area)

result = cv2.bitwise_and(img, img, mask=mask)


cv2.imshow('result', mask)
cv2.waitKey(100000)
cv2.destroyAllWindows()