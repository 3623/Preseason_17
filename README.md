# **Preseason 17**

## Visual Intertial Odometry
To figure out where we are in the world, we subconciously use our eyes to determine not only position relative to other objects, 
but also our speed relative to those objects.
It works quite well most of the time, and video game designers know this so that they can design their games to make it seem like we are moving.
Sometimes it screws us up, if we see the world surrounding us moving we will think that we are moving even when we aren't
(sometimes this happens when on a train). This is also evident when trying to balance on one leg with your eyes closed. 
We can apply this same concept to our robot. If the image in the camera starts moving, then the camera (which is attached to the robot) is moving.
That is what visual inertial odometry is, and like many other things that we do easily, computers do not.

### *goodFeaturesToTrack*
Designed to find corners of interest because they are easily trackable through multiple frames.
Corner detection algorithms are bad with reflective surfaces and lighting objects.
Because of this points jump around on test video.  
[OpenCV Docs on features](http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=cornerharris#cornerharris)  
Would be implemented by comparing location of each previous point to updated points in same vicinity.
Could also use estimated velocity to predict search location.
Rotation could be done similarly (I think) but woud be more complex (obviously)

### *Phase Correlation*
Uses fast (discrete) fourier transform algorithm to describe each image in a comparable way.
Does math to compute translation from one image to the next.  
[Wikipedia on Phase Correlation](https://en.wikipedia.org/wiki/Phase_correlation)  
[OpenCV Docs on DFT](http://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html#dft)

### *calcOpticalFlowPyrLK*
OpenCV's method of doing optical flow by the Lucas-Kanade Method, which assumes that motion will be the same in neighboring areas. 
Optical flow assumes that an image will retain the same intensity through multiple frames (auto-adjust/brightness must be turned off). 
OpenCV's method takes two input images (previous and updated) and the set of points which are returned from goodFeaturesToTrack.
The Lucas-Kanade Method implementation will track those points and return the location of new points in the update frame.  
This is basically a more robust implementation of goddFeaturesToTrack, as it can handle large movements because of its "pyramiding" algorithm.  
[OpenCV Docs on Optical Flow](http://docs.opencv.org/3.2.0/d7/d8b/tutorial_py_lucas_kanade.html)  
[More OpenCV Docs on Optical Flow](http://docs.opencv.org/3.2.0/d7/de9/group__video.html)  

### *Template Matching*
OpenCV provides multiple implementations of template matching.


## State Estimation
As we know subconciously, there are multiple ways to track the state (position) of our robot.
As basic drive/time auto works on this principle. 
In theory if the robot drives a set period of time it will move a set distance and end up in a set location.
As we also learned last year, this is not necessarily the case.
The robot drifts and collides with other things on the field.
We can also use IMU sensors (like the gyroscope built into the roboRio) but they too aren't perfect.
The gyroscope drifts after some time and has to be calibrated correctly at startup.
*Everything* has some error.
But what if we combined multiple measurements? 
With data from multiple methods to estimate state we can get a more accurate estimation.
This is what Kalman Filters and Alpha-Beta Filters are.

### *Alpha-Beta Filter* 

[Wikipedia on Alpha-Beta filters](https://en.wikipedia.org/wiki/Alpha_beta_filter)  
Since this is much simpler than Kalman filters to implement, and sometimes has better performance, we should implement this.
There are multiple things that can be implemented in this:
- Gyroscope reading (for direction)
- Magnetometer reading (for direction)
- Visual Inertial Odometry (for both direction and location)
- Encoders on drive train (for posibbly both direction and location)
- Accelerometers (for location)  

(We have a gyroscope (built into roboRio), and multiple accelerometers. 
We do not have a magnetometer however. 
We should look into investing in a proper IMU. 
[Heres a relatively cheap one.](https://www.vexrobotics.com/vexpro/motors-electronics/pigeon-imu.html))