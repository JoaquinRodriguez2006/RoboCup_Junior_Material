# from controller import Robot
import cv2
import numpy as np

def detectVisualSimple(image_data):

	coords_list = []
	# img = np.array(np.frombuffer(image_data, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)))
	# img[:,:,2] = np.zeros([img.shape[0], img.shape[1]])
	img = image_data
 	cv2.imshow('Primera Imagen', img)


	#convert from BGR to HSV color space
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	cv2.imshow('Imagen Grises', gray)
	#apply threshold
	thresh = cv2.threshold(gray, 140, 255, cv2.THRESH_BINARY)[1]
	cv2.imshow('Trhes', thresh)

	# draw all contours in green and accepted ones in red
	contours, h = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
 

	for c in contours:
		if cv2.contourArea(c) > 1000:
			coords = list(c[0][0])
			coords_list.append(coords)
			return ((int(coords[0])),int(coords[1]))

# robot = Robot()
timeStep = 32

# camera_centro = robot.getDevice('camera1')
# camera_centro.enable(timeStep)

img = 'Imagenes/imagenes rombo/imagenes_rombo_gasrombo_2.png'
print(detectVisualSimple(img))
cv2.waitKey(0)
""""
while robot.step(timeStep) != -1:
	img = camera_centro.getImage()
	print(detectVisualSimple(img, camera_centro))
	img = np.array(np.frombuffer(img, np.uint8).reshape((camera_centro.getHeight(), camera_centro.getWidth(), 4)))
	cv2.imshow("Imagen", img)
	img = 
	cv2.waitKey(1)
"""