# object-position-estimation-from-uav-video-
This repository is created for object position estimation from drone video.

Provided : 
- Drone Video
- Camera Extrinsic Parameters
  - T = (Tx, Ty, Tz) the position of the camera projection center in world coordinate system
  - R the rotation matrix that defines the camera orientation with angles ω, φ, κ 

Missings:
- Camera Intrinsic Parameters
  - Principal Point,Focal Lenght, Lens Distortions, Fiducial Marks   
- Ground Control Points 
- Digital Orthophoto is available for Berlin with 20 cm resolution and 40 cm position accuracy

Wanted:
- A World Coordinate of an object on the drone imagery 
