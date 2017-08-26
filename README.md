# Preseason_17

## Visual Intertial Nav

### goodFeaturesToTrack

Designed to find corners of interest because they are easily trackable through multiple frames.
Corner detection algorithms are bad with reflective surfaces and lighting objects.
Because of this points jump around on test video.
Would be implemented by comparing location of each previous point to updated points in same vicinity.
Could also use estimated velocity to predict search location.
Rotation could be done similarly (I think) but woud be more complex (obviously)