import cv2
import numpy as np
import glob

frameSize = (500, 500)

out = cv2.VideoWriter('a.avi',cv2.VideoWriter_fourcc(*'DIVX'), 60, frameSize)

for filename in glob.glob('C:/Users/JOAKOL/Desktop/robotica/Imagenes/mapa_all-02/centro_2/*.png'):
    img = cv2.imread(filename)
    out.write(img)

out.release()