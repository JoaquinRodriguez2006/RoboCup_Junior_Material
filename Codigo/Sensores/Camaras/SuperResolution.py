import cv2

img = 'Porsche/C:\Users\Neosys\Documents\Robótica 2022'
image = cv2.imread(img)

def InitSuperResolution(img):
    global net
    net = cv2.dnn.readNetFromONNX(img)

#def super_resolution(image, returndata=False):
    #