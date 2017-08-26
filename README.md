# Preseason_17

## Visual Intertial Nav

### goodFeaturesToTrack

Designed to find corners of interest because they are easily trackable through multiple frames.
Corner detection algorithms are bad with reflective surfaces and lighting objects.
Because of this points jump around on test video.  
[OpenCV Docs on features](http://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=cornerharris#cornerharris)  
Would be implemented by comparing location of each previous point to updated points in same vicinity.
Could also use estimated velocity to predict search location.
Rotation could be done similarly (I think) but woud be more complex (obviously)

### Phase Correlation

Uses fast (discrete) fourier transform algorithm to describe each image in a comparable way.
Does math to compute translation from one image to the next.  
[Wikipedia on Phase Correlation](https://en.wikipedia.org/wiki/Phase_correlation)