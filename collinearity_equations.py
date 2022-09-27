# Name:        Collinearity Equations  
# Purpose:     Position Estimation of an object from a frame
#
# Author:      Güray
# Additional
# source:         https://stackoverflow.com/questions/68195056/finding-the-real-world-coordinates-of-an-object-from-a-camera
# Collinearity Eq http://sar.kangwon.ac.kr/etc/rs_note/rsnote/cp9/cp9-6.htm
# Camera Calib    https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
# Camera Transform https://github.com/tanayrastogi/CameraTransform/blob/main/transform.py
# Created:     06/09/2022
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import numpy as np
import cv2
from pathlib import Path 
import os
from natsort import natsorted
import pandas as pd
from math import sin, cos
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from math import radians

data_path=str(Path.home())+r'/Documents/Drone_Fly/Flug_21_08_22'

scan_names = sorted(list(set([os.path.basename(x)[5:-4] for x in os.listdir(data_path+"/frames_100ms")])))

scan_names_sorted=natsorted(scan_names)

df = pd.read_csv(data_path+'/Aug-21st-2022-06-20PM-Flight-Airdata.csv')
eop_row=df.loc[df['time(millisecond)'] == int(scan_names_sorted[0])]


def extrensic_parameters():

    """
    
    eop = dict of exterior orientation parameters: omega, phi, kappa, XL, YL, ZL -- Bew aware that Depends on the camera, the coordinate system can be changed.
    Kappa (κ), the rotation around the Z axis.
    Phi (φ), the rotation around the Y axis.
    Omega (ω), the rotation around the Χ axis
    
    """

    eop={}
    
    eop['omega'] = radians(eop_row[' roll(degrees)'].squeeze())
    eop['phi'] = radians(eop_row[' pitch(degrees)'].squeeze())
    eop['kappa'] = radians(eop_row[' compass_heading(degrees)'].squeeze())    
    eop['XL']=eop_row['latitude'].squeeze()
    eop['YL']=eop_row['longitude'].squeeze()
    eop['ZL']=eop_row['altitude_above_seaLevel(feet)'].squeeze()
    
    # Calculate the Rotation Parameters
    
    om = eop['omega']
    ph = eop['phi']
    kp = eop['kappa']

    XL = eop['XL']
    YL = eop['YL']
    ZL = eop['ZL']


    Rom = np.matrix([[1, 0, 0], [0, cos(om), sin(om)], [0, -sin(om), cos(om)]])
    Rph = np.matrix([[cos(ph), 0, -sin(ph)], [0, 1, 0], [sin(ph), 0, cos(ph)]])
    Rkp = np.matrix([[cos(kp), sin(kp), 0], [-sin(kp), cos(kp), 0], [0, 0, 1]])
    
    T_X = []        # Translation x [meters]
    T_Y = []        # Translation y [meters]
    T_Z = []        # Translation z [meters]
    
    R = Rkp * Rph * Rom
    
    # Translation from camera to ground -- ??????
    T = np.array([[T_X], [T_Y], [T_Z]])
    
    return np.hstack((R, T))

def intirinsic_parameters():
    """
    iop = dict of interior orientation parameters: x0, y0, f - - Danny will bring the drone and we can make calibration to estimate the parameters.
    # https://github.com/KNITPhoenix/Calculating-intrinsic-parameters-of-a-camera  -- It can be used for calculation of intrinsic parameters.
    """
    # iop = {}
    
    # # data from lines 7-9 of the camera_file
    # iop['x0'] = dat[6]
    # iop['y0'] = dat[7]
    # iop['f'] = dat[8]
    
    fx = camera_params["img_width"]/camera_params["sen_width"]    # px/mm ## Relationship between px and mm
    fy =  camera_params["img_height"]/camera_params["sen_height"]  # px/mm ## Relationship between px and mm
    cx = camera_params["img_width"]/2                             # px    ## Center of the image in x
    cy = camera_params["img_height"]/2                            # px    ## Center of the image in y
    
    # Matrix
    K = np.array([[1/fx, 0,     -cx/fx],
                  [0,    1/fy,  -cy/fy],
                  [0,    0,          1]])
    transform = camera_params["focal_length"]*np.array([[0, -1, 0],
                                                        [-1, 0, 0],
                                                        [0,  0, 1]])
    return np.dot(transform, K)


"""
x = array of x photo coordinates of control points
y = array of y photo coordinates of control points
X = array of X world coordinates of control points
Y = array of Y world coordinates of control points
Z = array of Z world coordinates of control points
"""
X=400720.800
Y=5824483.000
Z=150

# Continue to adopt this codes to the script
# https://github.com/tanayrastogi/CameraTransform/blob/main/transform.py
# https://www.mathworks.com/help/vision/ref/worldtoimage.html
# https://www.youtube.com/watch?v=US9p9CL9Ywg
# https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

uvw = R * np.matrix([[X-XL], [Y-YL], [Z-ZL]])

def drawPointOnImage(image,point):
    image = mpimg.imread(data_path+"/frames_100ms/frame"+image+".jpg")
    
    #pts = np.array([[330,620],[950,620],[692,450],[587,450]]) -- in case having more than one point
    pts=np.array(point)

    plt.imshow(image)
    plt.plot(640, 570, "og", markersize=20)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="x", color="red", s=200)
    plt.show()

drawPointOnImage(scan_names_sorted[0],[[330,620],[950,620],[692,450],[587,450]])

camera_params= dict(img_width    = 1920,            # px ## Width of the image  
                        img_height   = 1080,            # px ## Height of the image
                        sen_width    = 5.18,            # mm ## Width of the sensor array  
                        sen_height   = 3.89,            # mm ## Height of the sensor array  
                        focal_length = 3.93)            # mm ## Focal length of the camera 