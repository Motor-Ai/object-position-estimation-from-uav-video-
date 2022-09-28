# Object Position Estimation from Drone Video

This repository is created for object position estimation from drone video.

To estimate a position of an object in the image, collinearity equations for a single shot are being developed. 

In order to estimate XY real-world coordinates from a single image, the followings should be known:

X0, Y0, Z0 real-world coordinates of the camera
Exterior orientation of your camera (Pitch, Roll, and Yaw)
Interior orientation of your camera (focal length, principle point in x and y direction, radial and tangential distortion parameters)

For this task what we have : 
- Drone Video
- Camera Extrinsic Parameters
  - T = (Tx, Ty, Tz) the position of the camera projection center in world coordinate system
  - R the rotation matrix that defines the camera orientation with angles ω, φ, κ 

Missings:
- Camera Intrinsic Parameters
- Ground Control Points 
  - Digital Orthophoto is available for Berlin with 20 cm resolution and 40 cm position accuracy

Wanted:
- A World Coordinate of an object on the drone imagery 


The camera position and exterior parameters of the camera are provided for each frame, however,  the camera is not metric.
That means the intrinsic parameters of the camera are not known. Images are subject to distortions due to the lens or other external issues.

In this case, the camera should be calibrated. 
After that, we would have extrinsic and intrinsic parameters of the camera and we are able to compute between the real-world coordinates and the image coordinates of the selected objects in the scene.
