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
Knowing the distance from the camera to whatever is in frame (`D`)
(in our case the ceiling (to keep things 2 dimensional and thus simple)).
With that and the field of view (`FOV`)for the camera (which should be found on any specifications sheet) it would be simple trigonometry.
We could find the distance that every pixel represents (`S`) with `2D*cos(90-(FOV/2))` and the movement between frame in pixels (`P`), 
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
OpenCV's optical flow function can reliably return the vectors of each point tracked. 
With this the rotation and translation of the robot have to be determined.
The general form of an equation calculating the transformation of a point `(x,y)` is `x' = xcosθ-ysinθ+X` and `y' = ycosθ+xsinθ+Y`.
However we are trying to find the amount of rotation and amount of translation,which are described by `θ`, `X`, and `Y` respectively.
With three unknowns for both equations, those variables can be found using matrices if three points and their translations are known.
However, often times we will be tracking more than three points and how should three points be selected out of many?
Also often enough there will not be three points to track.   
[Physics Forum on this method](https://www.physicsforums.com/threads/trying-to-derive-a-transformation-matrix-from-a-set-of-known-points.360963/)  
Another way those variables can be solved for is by calculating the [homogenous transformation matrix](http://planning.cs.uiuc.edu/node99.html) (T), 
which is a matrix that contains the equations listed previously.
```
 |cosθ   -sinθ     X|    |x1  x2 .. xN|    |x'1  x'2 .. x'N|
T|sinθ    cosθ     Y| x X|y1  y2 .. yN|= X'|y'1  y'2 .. y'N|
 | 0        0      1|    | 1   1 ..  1|    |  1    1 ..   1|
```
This can be multiplied by a set of points (`X`) to get the translated points (`X'`).
However, as said before, we know the original and translated points, and need to find the transformation matrix.
We can solve this equation for the transformation matrix quite easily.
In normal math, the equation `T*X=X'` would be solved for `T` by dividing by `X` on both sides of the equation.
However division does not exist in matrix math, instead multiplication of a matrix by its inverse is done.
`X` multiplied by its inverse, would result an identity matrix (`I`), which essentially is equal to 1 is normal algebra.
Therefore, by multiplying both sides of the equation by the inverse of `X`, we get `T*I=X*Xinv'`.
Therefore, the transformation matrix can be calculated by `T=X*Xinv`
> Those familiar with matrices woud notice a couple of wierd things in that equation.
The first one is that the set of points we have is not necessarily what is called a square matrix, 
which is a matrix that has the same amount of rows and columns.
Generally speaking, a matrix has to be square in order to find its identity, for various reasons, 
the main being that an inverse is defined that any matrix multiplied by its inverse in any order (`A*Ainv` or `Ainv*A`) 
results in an Identity matrix (also generally speaking, the order of multiplication matters, and `AB` is not `BA`, 
except in this case of inverses, where it should be).
However, something called a psuedo-inverse can be calculated, 
which essentially finds the inverse of a matrix if multiplied in a specific order (either `A*Ainv` or `Ainv*A`).
For this the *numpy* algorithm `pinv` is used.
The second wierd thing is that in matrix math, the order of a matrix multiplied by an indentity matrix also matters.
Generally, an identity matrix is defined that `I*A=A`, not `A*I=A` like we have in our equation.
However, because our matrix T is square, `T*I` is still equal to `T`.  

With the  transformation matrix, values of `θ`, `X`, and `Y` can be easily calculated.

This method, still needs multiple points to calculate reliably an accurate transformation matrix. 
For this a better method for chosing points to track is needed. 
Ideally, there are at least 3 distinct points to track, at minimum 2. 
If there are less, it is not even worth to track as any calculations done are most likely not accurate. 
Frame to frame, there are two types of points that should be tracked. 
The first are the points that were previously tracked, as we know that those points were at one time good features to track. 
However, in motion these points eventually move out of frame. 
The second type are new points that need to be picked up to compensate for the previous points leaving the image.
Implementing an efficient, elegant method for retaining previous points yet aquiring new points is essential. 
Additionally, there are fallback ways to chose points if not enough are chosen by previously said method. 
One way is to just lower the threshold for what a good point is. 
Another way is to pick points randomly or gridlike in the image, and hope that the returned tracked points are accurate, 
or a different tracking parameter set is used that removes points with low "scores."

---

## Vision
http://www.intorobotics.com/how-to-detect-and-track-object-with-opencv/

---