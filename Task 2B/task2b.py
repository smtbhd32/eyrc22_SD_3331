import numpy as np
import cv2

img = cv2.imread("yellow_detect.jpeg", -1)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_ylow = np.array([20, 100, 200])
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
print(x, y)
cv2.destroyAllWindows()