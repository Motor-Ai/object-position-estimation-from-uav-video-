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


def extrensic_parameters(eop_row):

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

def reconstruct3dfrom2dPixel():
    # https://stackoverflow.com/questions/11334012/reconstruct-3d-coordinates-in-camera-coordinate-system-from-2d-pixels-with-sid
    # Camera Calibration Matrix
    """ K = [ 1.017215234570303 , 0 , 3.195000000000000 , 0 ;
             0 , 1.012549014668498 , 2.395000000000000 , 0 ;
             0 , 0 , 1.0000 , 0 ;
             0 , 0 , 0 ,0 ]"""
    K=intirinsic_parameters() # add 0,0,0,1 to the last column
    
    #Calculate the 3D to 2D - Projection Matrix P (3X4)
    
    P=K*np.hstack((np.eye(3),np.zeros((3, 1))))
    
    # When converting a Point in World-Coordinates [X, Y, Z]_World, transform it first to Camera-Coordinates and then project it to 2D:
    
    P_world = np.array([X, Y, Z, 1])
    
    P_camera=R_world_to_Camera*P_world
    
    # Projection
    
    P_pixels=P*camera
    
    P_pixels = P_pixels / P_pixels[3]  # normalize coordinates
    
    return P_pixels
    
def backpropagation(World_Coor):
    
    
    # The camera is aligned in such a way that it is approx. in the direction of the pos. X-axis of world Kosy
    # shows. In the camera image, the pos. y-axis then to the left, the pos. z-axis up.
    R_World_to_Cam=extrensic_parameters()
    
    K=intirinsic_parameters()
    
    # projection and transformation matrix P
    
    P = K * R_World_to_Cam
    
    # Any X_World points in the world coordinate system [mm] (homogeneous coordinates)

    X_World = World_Coor
    
    # Transform and project 3D World -> 2D Picture
    
    X_Pic_1 = P * X_World;
    
    # normalize the homogeneous coordinates (3rd element must be 1!)
    
    X_Pic_1 = X_Pic_1 / X_Pic_1 ( 3 )
    
    # Now take any four pixels from the camera image...
    # (just 30px to the right and 40px down calculated from the top)
    
    X_Pic_backtransform_1 = X_Pic_1[1 : 30 ]+ [ 30 , 40 , 0 ]
    
    # ... and transform them back according to Ilker Savas, "Development of a system
    # for the visual positioning of interaction partners"
    M_Mat = P [ 1 : 3 , 1 : 3 ] ;      #Matrix M is the "top-front" 3x3 part
    p_4 = P [ 1 : 3 , 4 ];              # Vector p_4 is the "top-rear" 1x3 part
    C_tilde = - np.linalg.inv(M_Mat) * p_4;     # calculate C_tilde

    # Invert Projection with Side-Condition ( Z = 0 ) and Transform back to
    # World-Coordinate-System
    X_Tilde_1 ​​= np.lingalg.inv(M_Mat) * X_Pic_backtransform_1
    mue_N_1 = -C_tilde [3] / X_Tilde_1 [3]
       
    #Do the inversion of above steps...
    X_World_backtransform_1 = mue_N_1 * numpy.linalg.inv ( M_Mat )* X_Pic_backtransform_1 + C_tilde
       
    return X_World_backtransform_1

def collinearity coordinates(pixelcoord="True"):
    """
    Undistort your images
    Create image coordinates of the points you wish to know the real world coordinate of, use undistorted image.
    Create rotation matrices with rotations along X,Y,Z axis and do matrix multiplikation on them (RXRYRZ)
    Apply collinearity equations 
    """   
    # https://github.com/tanayrastogi/CameraTransform/blob/main/transform.py
    # https://www.mathworks.com/help/vision/ref/worldtoimage.html
    # https://www.youtube.com/watch?v=US9p9CL9Ywg
    # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    #https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points
    # Undistort the image 
    #http://sar.kangwon.ac.kr/etc/rs_note/rsnote/cp9/cp9-6.htm""
    
    r=extrensic_parameters()
    
    """
    projection center or lens be 0 (X0, Y0, Z0), with rotation angles ,around X, Y and Z axis respectively (roll, pitch and yaw angles), 
    the image coordinates be p (x,y) and the ground coordinates be P(X,Y, Z)
    """
    if pixelcoord=="True"    :
        
        x=-f*((r[0,0]*(X-X0)+r[0,1]*(Y-Y0)+r[0,2]*(Z-Z0))/(r[3,0]*(X-X0)+r[3,1]*(Y-Y0)+r[3,2]*(Z-Z0)))
        Y=-f*((r[1,0]*(X-X0)+r[1,1]*(Y-Y0)+r[1,2]*(Z-Z0))/(r[3,0]*(X-X0)+r[3,1]*(Y-Y0)+r[3,2]*(Z-Z0)))
    
    else:
        """
        In the case of a camera, the previous formula includes six unknown parameters 
        (X0,Y0,Z0 ; , , ) which can be determined with the use of more than three ground control points (Xi,Yi; Xi,Yi,Zi). The collinearity equation can be inversed as follows-
        """
        
        X=(Z-Z0)*((r[0,0]*x+r[1,0]*y+r[2,0]*f)/(r[1,1]*(X-X0)+r[1,2]*(Y-Y0)+r[2,2]))+X0
        Y=(Z-Z0)*((r[0,1]*x+r[1,1]*y+r[2,1]*f)/(r[1,1]*(X-X0)+r[1,2]*(Y-Y0)+r[2,2]))+Y0

def drawPointOnImage(image,point):
    
    image = mpimg.imread(data_path+"/frames_100ms/frame"+image+".jpg")
    
    #pts = np.array([[330,620],[950,620],[692,450],[587,450]]) -- in case having more than one point
    pts=np.array(point)

    plt.imshow(image)
    plt.plot(640, 570, "og", markersize=20)  # og:shorthand for green circle
    plt.scatter(pts[:, 0], pts[:, 1], marker="x", color="red", s=200)
    plt.show()


if __name__ == '__main__':
    
    drawPointOnImage(scan_names_sorted[0],[[330,620],[950,620],[692,450],[587,450]])
    
    
    camera_params= dict(img_width    = 1920,            # px ## Width of the image  
                        img_height   = 1080,            # px ## Height of the image
                        sen_width    = 5.18,            # mm ## Width of the sensor array  
                        sen_height   = 3.89,            # mm ## Height of the sensor array  
                        focal_length = 3.93)            # mm ## Focal length of the camera 
    # Any X_World points in the world coordinate system [mm] (homogeneous coordinates)

    X_World = [ 20000 , 2000 , 0 , 1 ] 