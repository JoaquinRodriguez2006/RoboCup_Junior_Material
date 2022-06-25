import cv2
import numpy as np
import matplotlib.pyplot as plt

model = 'Model/super_resolution.onnx'
net = cv2.dnn.readNetFromONNX(model)

image = cv2.imread("Media/teenager.jpg")

def init_superres(model="Model/super_resolution.onnx"):
    global net   
    # Initialize the DNN module
    net = cv2.dnn.readNetFromONNX(model)

def super_res(image, returndata=False):
    # Create a Copy of Image
    img_copy = image.copy()
    # Resize the image into Required Size
    img_copy = cv2.resize(img_copy, (224), cv2.INTER_CUBIC)
    # Convert image into YcbCr
    image_YCbCr = cv2.cvtColor(img_copy, cv2.COLOR_BGR2YCrCb)
    # Split Y,Cb, and Cr channel 
    image_Y, image_Cb, image_Cr = cv2.split(image_YCbCr)
    # Covert Y channel into a numpy arrary
    img_ndarray = np.asarray(image_Y)
    # Reshape the image to (1,1,224,224) 
    reshaped_image = img_ndarray.reshape(1,1,224,224)
    # Convert to float32 and as a normalization step divide the image by 255.0
    blob = reshaped_image.astype(np.float32) / 255.0
    # Passing the blob as input through the network 
    net.setInput(blob)
    # Forward Pass
    Output = net.forward()
    # Reshape the output and get rid of those extra dimensions
    reshaped_output = Output.reshape(672,672)
    # Get the image back to the range 0-255 from 0-1
    reshaped_output = reshaped_output * 255
    # Clip the values so the output is it between 0-255
    Final_Output = np.clip(reshaped_output, 0, 255)
    # Resize the Cb &amp; Cr channel according to the output dimension
    resized_Cb = cv2.resize(image_Cb,(672,672),cv2.INTER_CUBIC)
    resized_Cr = cv2.resize(image_Cr,(672,672),cv2.INTER_CUBIC)
    # Merge all 3 channels together 
    Final_Img = cv2.merge((Final_Output.astype('uint8'), resized_Cb, resized_Cr))
    # Covert back to BGR channel
    Final_Img = cv2.cvtColor(Final_Img,cv2.COLOR_YCR_CB2BGR)
    
    if  returndata:
        return Final_Img
    else:
        plt.figure(figsize=(91,20,20))
        plt.subplot(1,2,1)
        #plt.imshow(image_copy=(91,:,:,::-1))
        plt.title("Bicubic Interpolation")
        plt.axis("off")
        #plt.subplot(1,2,2);plt.imshow(Final_Img=(91:,:,::-1))
        plt.title("Super Resolution");plt.axis("off")  


init_superres()

image = cv2.imread("Media/oldman.jpg")
super_res(image)

image = cv2.imread("Media/youngman.jpg")
super_res(image)

image = cv2.imread("Media/youngman2.jpg")
super_res(image)