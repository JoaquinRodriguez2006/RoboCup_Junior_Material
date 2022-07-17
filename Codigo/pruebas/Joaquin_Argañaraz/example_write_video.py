
import cv2

#Arreglo vacío
img_array = []

#For para leer imagenes desde un directorio
for x in range(0, 3320):
    path = f'C:/Users/JOAKOL/Desktop/robotica/Imagenes/mapa_all-02/izquierda/imagen_webot_{x}_mapa_all-02.png'
    img = cv2.imread(path)
    img_array.append(img)

#Tamaño de la última imagen alto y ancho
height, width = img.shape[:2]

video = cv2.VideoWriter(
    'mapa_all-02_izquierda.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 31, (width, height))

#For para guardar frames en un video
for i in range(len(img_array)):
    video.write(img_array[i])

print("Video creado")
video.release()
