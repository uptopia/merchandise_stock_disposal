import numpy as np

import cv2
import cv2.aruco as aruco

import matplotlib.pyplot as plt
import matplotlib as mpl

markerLength = 0.015
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_250)

# markerID = [#plum_riceball(5)
#             10, 11, 12, 13, 14, 
#             20, 21, 22, 23, 24,
#             30, 31, 32, 33, 34,
#             40, 41, 42, 43, 44,
            
#             #salmon_riceball(5)
#             50, 51, 52, 53, 54,
#             60, 61, 62, 63, 64,
#             70, 71, 72, 73, 74,
#             80, 81, 82, 83, 84,
            
#             #sandwich(5)
#             90, 91, 92, 93, 94,
#             100, 101, 102, 103, 104,
#             110, 111, 112, 113, 114,
            
#             #hamburger(2)
#             120, 121,
#             130, 131,
#             140, 141,
            
#             #drink(4)
#             150, 151, 152, 153, 154,
#             160, 161, 162, 163, 164,
#             170, 171, 172, 173, 174,                

#             #lunchbox(2)
#             180, 181,
#             190, 191,
#             200, 201,
#             
#             #shelf 
#             210, 211,
#             220, 221
#             230, 231]



if __name__ == '__main__':
    markerSize_mm = 16
    markerSize_inch = markerSize_mm / 25.4 #1 inch = 25.4 mm
    printer_dpi = 300 #dot per inch
    markerSize_pixel = 3200 #printer_dpi*markerSize_inch

    print('markerSize_mm', markerSize_mm)
    print('markerSize_inch', markerSize_inch)
    print('printer_dpi', printer_dpi)
    print('markerSize_pixel', markerSize_pixel)

    # # 1 inch = 72 points
    # # 1 inch = 25.4 mm
    # # 1 point = 0.352777778 mm

    markerID = []
    for i in range(13, 26):        #(1,13)(13, 26)
        for j in range(0, 5):
            # print(i*10+j)
            markerID.append(i*10+j)
        
    print('total {} markers IDs:\n {}'.format(len(markerID), markerID))
   
       
    nx = 10
    ny = int(len(markerID)/nx)
    print(nx, ny)

    fig = plt.figure()
    for i in range(1, nx*ny+1):        
        ax = fig.add_subplot(ny, nx, i)        
        img = aruco.drawMarker(ARUCO_DICT, markerID[i-1], markerSize_pixel) # aruco::drawMarker(dictionary, markerId, markerSize(pixel), markerImg, borderBits);        
        plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
        ax.axis("off")

    plt.savefig("markersB1.pdf")
    # plt.savefig("markers.png")
    plt.show()
