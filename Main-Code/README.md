## **State Estimation**
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
[Wikipedia MHE](https://en.wikipedia.org/wiki/Moving_horizon_estimation)  

### *Alpha-Beta Filter* 
- [ ] Cover general theory  
[Wikipedia on Alpha-Beta filters](https://en.wikipedia.org/wiki/Alpha_beta_filter)  
Since this is much simpler than Kalman filters to implement, and sometimes has better performance, we should implement this.
There are multiple things that can be implemented in this:
- Gyroscope reading (for direction)
- Magnetometer reading (for direction)
- [Visual Inertial Odometry](/Vision) (for both direction and location)
- Encoders on drive train (for posibbly both direction and location)
- Accelerometers (for location)  

>We have a gyroscope (built into roboRio), and multiple accelerometers. 
We do not have a magnetometer however. 
We should look into investing in a proper IMU. 
[Heres a relatively cheap one.](https://www.vexrobotics.com/vexpro/motors-electronics/pigeon-imu.html)

https://github.com/RobotCasserole1736/RobotCasserole2017
https://github.com/ligerbots/Steamworks2017Robot
https://github.com/246overclocked/hungryhippo
https://github.com/Team303/Java-Robot-Project-2017
https://github.com/Doodleman360/WaterGameConfirmed
