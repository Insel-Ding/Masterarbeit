import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
img = cv.imread('ablauf.drawio.png',cv.IMREAD_GRAYSCALE)
kernel = np.ones((5,5),np.float32)/25
median = cv.medianBlur(img,5)
plt.subplot(121),plt.imshow(img),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(median),plt.title('median')
plt.xticks([]), plt.yticks([])
plt.show()