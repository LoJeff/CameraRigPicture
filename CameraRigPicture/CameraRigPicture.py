
import serial
import time
import keyboard
import cv2
import numpy as np

#total number of pics expected
numPics = 1000

#Colour bounds in HSV
h_sens_g = 30
lower_green = np.array([60 - h_sens_g, 100, 100], dtype=np.uint8)
upper_green = np.array([60 + h_sens_g, 255, 255], dtype=np.uint8)

if __name__ == "__main__":
    cv2.namedWindow("vid")
    cam = cv2.VideoCapture(2)
    print("Preparing to take pictures (" + str(numPics) + ")")
    count = 0
    ret = True
    height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)

    ser = serial.Serial('COM8', 9600)
    print("connected")

    while count != numPics and ret:
        #Video display
        ret, frame = cam.read()
        frame = cv2.flip(frame, 1)

        #cropping image based on colour
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blur_hsv_frame = cv2.medianBlur(hsv_frame, 7)
        mask = cv2.inRange(blur_hsv_frame, lower_green, upper_green)
        inv_mask = cv2.bitwise_not(mask)
        filtered = cv2.bitwise_and(frame, frame, mask = inv_mask)

        
        #clean mask
        clean_inv_mask = cv2.fastNlMeansDenoising(inv_mask, None, 10, 10, 7)

        #increment number of pictures taken
        count += 1

        #Change alphas and find crop coordinates
        cropCoord = [[width, height], [0, 0]] # top left coords, bottom right coords
        bgra_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
        y, x = np.shape(clean_inv_mask)
        for j in range(0,y):
            for i in range(0,x):
                if (mask[j][i] != 0):
                    bgra_frame[j][i][3] = 0
                else:
                    #Finding the smallest square to fit the whole object
                    if i < cropCoord[0][0]:
                        cropCoord[0][0] = i
                    if i > cropCoord[1][0]:
                        cropCoord[1][0] = i
                    if j < cropCoord[0][1]:
                        cropCoord[0][1] = j
                    if j > cropCoord[1][1]:
                        cropCoord[1][1] = j

        #Crop Image
        crop_bgra_frame = bgra_frame[cropCoord[0][1]:cropCoord[1][1], cropCoord[0][0]:cropCoord[1][0]]

        #Saving image
        img_name = "posColour\pos_{}.png".format(count)
        cv2.imwrite(img_name, crop_bgra_frame1)
        print("{} written".format(img_name))

    cam1.release()
    cv2.destroyAllWindows()

    