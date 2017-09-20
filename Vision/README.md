## **Visual Intertial Odometry**
To figure out where we are in the world, we subconciously use our eyes to determine not only position relative to other objects, 
but also our speed relative to those objects.
It works quite well most of the time, and video game designers know this so that they can design their games to make it seem like we are moving.
Sometimes it screws us up, if we see the world surrounding us moving we will think that we are moving even when we aren't
(sometimes this happens when on a train). This is also evident when trying to balance on one leg with your eyes closed. 
We can apply this same concept to our robot. If the image in the camera starts moving, then the camera (which is attached to the robot) is moving.
That is what visual inertial odometry is, and like many other things that we do easily, computers do not.

- ### [*goodFeaturesToTrack*](feature_tracking_test.py)
Designed to find corners of interest because they are easily trackable through multiple frames.
Corner detection algorithms are bad with reflective surfaces and lighting objects.
Because of this points jump around on test video.  
[OpenCV Docs on features](http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=cornerharris#cornerharris)  
Would be implemented by comparing location of each previous point to updated points in same vicinity.
Could also use estimated velocity to predict search location.
Rotation could be done similarly (I think) but woud be more complex (obviously)

- ### [*Phase Correlation*](fast_fourier.py)
Uses fast (discrete) fourier transform algorithm to describe each image in a comparable way.
Does math to compute translation from one image to the next.  
[Wikipedia on Phase Correlation](https://en.wikipedia.org/wiki/Phase_correlation)  
[OpenCV Docs on DFT](http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#dft)

- ### [*calcOpticalFlowPyrLK*](opencv_optical_flow.py)
OpenCV's method of doing optical flow by the Lucas-Kanade Method, which assumes that motion will be the same in neighboring areas. 
Optical flow assumes that an image will retain the same intensity through multiple frames (auto-adjust/brightness must be turned off). 
OpenCV's method takes two input images (previous and updated) and the set of points which are returned from goodFeaturesToTrack.
The Lucas-Kanade Method implementation will track those points and return the location of new points in the update frame.  
This is basically a more robust implementation of goddFeaturesToTrack, as it can handle large movements because of its "pyramiding" algorithm.  
[OpenCV Docs on Optical Flow](http://docs.opencv.org/3.2.0/d7/d8b/tutorial_py_lucas_kanade.html)  
[More OpenCV Docs on Optical Flow](http://docs.opencv.org/3.2.0/d7/de9/group__video.html)  

- ### *Template Matching*
Template matching would be a crude way of determining the shift between frame to frame. 
The basic concept is that we would search in the updated frame for parts of the old image. 
If we search for corners from the old image, we are guaranteed a full match of one of the corner templates
(unless the image shifts more than the diagonal of the image). 
OpenCV provides multiple implementations of template matching.
Some methods can compensate for rotation. 
However all methods need a distinct template. 
We are guaranteed no such thing grabbing templates from the corners of each image. 
There is a high chance of false detections which would throw off the estimation of movement imensely. 
This doesn't seem to be a practical or advantageous solution.  
[Open CV Docs on Template Matching](http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_table_of_contents_feature2d/py_table_of_contents_feature2d.html)

----

All of these methods are used to determine the movement from one frame to the next.
With that we can do some math to determine the "actual" movement of the camera and thus the robot.
Knowing the distance from the camera to whatever is in frame (D)
(in our case the ceiling (to keep things 2 dimensional and thus simple)).
With that and the field of view (FOV)for the camera (which should be found on any specifications sheet) it would be simple trigonometry.
We could find the distance that every pixel represents (S) with `2D*cos(90-(FOV/2))` and the movement between frame in pixels (P), 
we could find the movement of the camera with `P*S`.

### Why use Visual Inertial Odometry?
Very precise sensors which can pick up very small movement are often expensive, tedious, and fragile. 
Many of the sensors we have access to are just not good enough to pick up the slow drift we would have and had with holonomic drive.
A camera however, is cheap and reflects accuratelly small shifts in movement and if properly vibration-isolated, can give an accurate reflection of this too.
For the purpose of small drift and movement the camera can be very accurate. 
Of course at fast and faster speeds the camera needs to have a high enough frame rate to be able to capture images with at least some similar parts. 
Therefore we would want to use an algorithm that can as accurately as possible detect movement from frame to frame, 
and does this fast. For that, I think dealing with OpenCV's optical flow method works well.
The problem is now to deal with the different movements of multiple points and to be able to draw conclusions about the rotational and linear movement of our robot. 

## OpenCV Optical Flow (In Depth)

---

## Vision
http://www.intorobotics.com/how-to-detect-and-track-object-with-opencv/

---