import cv2
from matplotlib import pyplot as plt
import numpy as np


img = plt.imread('./TMU.jpg')
print(img)
img.tofile('data.csv',sep=',')
