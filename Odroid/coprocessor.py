#! /usr/bin/env python

import rospy
import time
import math

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from ardrone_autonomy.msg import Navdata  # message type for nav_callback
from control_lab.msg import EstimatedState

from Queue import PriorityQueue


class Filter:
    def __init__(self):
        self.baseline_height = 0

    def update_with_navdata(self, state, navdata):
        (alpha, beta) = (.25, 0.001)
        dt = navdata.header.stamp.to_sec() - state.header.stamp.to_sec()
        if dt == 0:
            return state

        state = self.project(state, navdata.header.stamp)

        ## Height Estimate
        obs_alt = navdata.altd / 1000.0 - self.baseline_height
        if abs(obs_alt - state.z) < .15:
            state.z += alpha * (obs_alt - state.z)
        else:
            self.baseline_height += obs_alt - state.z

        ## Velocity Estimates
        angle = -state.yaw * 3.1415 / 180.0
        obs_vy = math.cos(angle) * navdata.vy / 1000.0 - math.sin(angle) * navdata.vx / 1000.0
        obs_vx = math.sin(angle) * navdata.vy / 1000.0 + math.cos(angle) * navdata.vx / 1000.0
        obs_vz = navdata.vz / 1000.0

        state.dx += alpha * (1.2 * obs_vx - state.dx)
        state.dy += alpha * (1.2 * obs_vy - state.dy)
        state.dz += alpha * (obs_vz - state.dz)

        # RPY Estimates
        obs_roll = navdata.rotX
        obs_pitch = navdata.rotY
        obs_yaw = navdata.rotZ

        state.roll += alpha * (obs_roll - state.roll)
        state.pitch += alpha * (obs_pitch - state.pitch)

        err_yaw = obs_yaw - state.yaw_drone

        state.yaw_drone += alpha * err_yaw

        # state.dyaw += beta/dt*err_yaw

        # Drone State and Battery
        state.droneState = navdata.state
        state.batteryPercent = navdata.batteryPercent
        return state

    def update_with_vel(self, state, vel):
        (alpha, beta) = (0.5, 0.5)
        state = self.project(state, vel.header.stamp)

        return state

    def update_with_location(self, state, location):
        (alpha, beta) = (0.4, 0.00005)

        dt = location.header.stamp.to_sec() - state.header.stamp.to_sec()
        if dt == 0:
            return state
        state = self.project(state, location.header.stamp)

        err_x = location.twist.linear.x - state.x
        err_y = location.twist.linear.y - state.y
        err_z = location.twist.linear.z - state.z
        err_yaw = location.twist.angular.z * 180.0 / 3.1415 - state.yaw

        state.x += alpha * err_x
        state.y += alpha * err_y
        state.z += alpha * err_z
        state.yaw += alpha * err_yaw
        state.yaw_offset = state.yaw - state.yaw_drone

        state.dx += beta / dt * err_x
        state.dy += beta / dt * err_y
        state.dz += beta / dt * err_z

        return state

    def project(self, state, t):
        dt = t.to_sec() - state.header.stamp.to_sec()

        state.x += state.dx * dt
        state.y += state.dy * dt
        state.z += state.dz * dt

        state.yaw_drone += state.dyaw * dt
        state.yaw = state.yaw_drone + state.yaw_offset

        state.header.stamp = t
        return state


class Estimator:
    def __init__(self, filt):
        self.filter = filt

        # Specify relevant ROS topics
        nav_topic = "/ardrone/navdata"  # Flight Data
        vel_topic = "/cmd_vel"  # Velocity Commands
        qr_topic = "/location"  # QR Code Localization
        output_topic = "/estimated_state"  # Pose Estimate

        # Parameters to handle timestamps
        self.lastNavStamp = rospy.Time(0)
        self.lastDroneTS = 0
        self.droneRosTSOffset = 0

        self.lastEstimatedState = EstimatedState()
        self.lastEstimatedState.header.stamp = rospy.Time.now()
        self.queue = PriorityQueue()

        # Publish pose estimate
        self.pub = rospy.Publisher(output_topic, EstimatedState, queue_size=1)

        # Subscribe to data feeds
        rospy.Subscriber(nav_topic, Navdata, self.nav_callback)
        rospy.Subscriber(vel_topic, Twist, self.vel_callback)
        rospy.Subscriber(qr_topic, TwistStamped, self.qr_callback)

    def nav_callback(self, nav_msg):
        lastNavdataReceived = nav_msg

        if (rospy.Time.now() - lastNavdataReceived.header.stamp) > rospy.rostime.Duration(30.0):
            lastNavdataReceived.header.stamp = rospy.Time.now()

        # darn ROS really messes up timestamps.
        # they should arrive every 5ms, with occasionally dropped packages.
        # instead, they arrive with gaps of up to 30ms, and then 6 packages with the same timestamp.
        # so: this procedure "smoothes out" received package timestamps, shifting their timestamp by max. 20ms to better fit the order.
        rosTS = self.getMS(lastNavdataReceived.header.stamp)
        droneTS = lastNavdataReceived.tm / 1000

        if self.lastDroneTS == 0:
            self.lastDroneTS = droneTS

        if (droneTS + 1000000) < self.lastDroneTS:
            self.droneRosTSOffset = rosTS - droneTS  # timestamp-overflow, reset running average.
            rospy.logwarn(
                "Drone Navdata timestamp overflow! (should happen epprox every 30min, while drone switched on)")
        else:
            self.droneRosTSOffset = 0.9 * self.droneRosTSOffset + 0.1 * (rosTS - droneTS)

        rosTSNew = droneTS + self.droneRosTSOffset  # this should be the correct timestamp.
        TSDiff = min(1001, max(-1001, rosTSNew - rosTS))  # never change by more than 100ms.
        lastNavdataReceived.header.stamp += rospy.rostime.Duration(TSDiff / 1000.0)  # change!
        lastRosTS = rosTS
        self.lastDroneTS = droneTS

        # save last timestamp
        self.lastNavStamp = lastNavdataReceived.header.stamp

        # push back in filter queue.
        self.queue.put((lastNavdataReceived.header.stamp.to_sec(), ('navdata', lastNavdataReceived)))

    def vel_callback(self, vel_msg):
        ts = TwistStamped()
        ts.header.stamp = rospy.Time.now()
        ts.twist = vel_msg
        self.queue.put((ts.header.stamp.to_sec(), ('vel', ts)))

    def qr_callback(self, qr_msg):
        self.queue.put((qr_msg.header.stamp.to_sec(), ('location', qr_msg)))

    def run(self):
        r = rospy.Rate(100)  # 10 hz
        time.sleep(1)
        self.lastEstimatedState.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
            currentTime = rospy.Time.now()

            # work through queue and update state
            while not self.queue.empty():
                (t, (datatype, data)) = self.queue.get()
                if rospy.Time.to_sec(currentTime) < t:
                    self.queue.put((t, (datatype, data)))
                    break
                elif datatype == 'navdata':
                    self.lastEstimatedState = self.filter.update_with_navdata(self.lastEstimatedState, data)
                elif datatype == 'vel':
                    self.lastEstimatedState = self.filter.update_with_vel(self.lastEstimatedState, data)
                elif datatype == 'location':
                    self.lastEstimatedState = self.filter.update_with_location(self.lastEstimatedState, data)

            # project state to current time
            self.lastEstimatedState = self.filter.project(self.lastEstimatedState, currentTime)

            # Publish
            self.pub.publish(self.lastEstimatedState)

            r.sleep()

    def getMS(self, stamp):
        ros_header_timestamp_base = stamp.secs
        mss = (stamp.secs - ros_header_timestamp_base) * 1000 + stamp.nsecs / 1000000
        return mss


if __name__ == '__main__':
    estimator = Estimator(Filter())
    print 'Estimation Running'
    estimator.run()
# API Training Shop Blog About

