package org.usfirst.frc.team1736.robot;

/*
 *******************************************************************************************
 * Copyright (C) 2017 FRC Team 1736 Robot Casserole - www.robotcasserole.org
 *******************************************************************************************
 *
 * This software is released under the MIT Licence - see the license.txt
 *  file in the root of this repo.
 *
 * Non-legally-binding statement from Team 1736:
 *  Thank you for taking the time to read through our software! We hope you
 *   find it educational and informative! 
 *  Please feel free to snag our software for your own use in whatever project
 *   you have going on right now! We'd love to be able to help out! Shoot us 
 *   any questions you may have, all our contact info should be on our website
 *   (listed above).
 *  If you happen to end up using our software to make money, that is wonderful!
 *   Robot Casserole is always looking for more sponsors, so we'd be very appreciative
 *   if you would consider donating to our club to help further STEM education.
 */

import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.lib.SignalMath.InterpValueHistoryBuffer;

import edu.wpi.first.wpilibj.Timer;

public class VisionAlignment {
	private static VisionAlignment visionAlignment = null;

	// Timer for sDelay state
	Timer delayTimer;

	private VisionAlignAnglePID anglePID;

	// Record of the history of gyro and distance values
	InterpValueHistoryBuffer gyroHistory;

	// Tolerances
	private double angleTol = 1.0;
	private double angleTolHyst = 0.025;// get within half a degree lined up
	private double gyroAngleDesiredLastFrame = 0;
	private double gyroAngleLastFrame = 0;

	// Keep track of what the most recent frame received from the coprocessor was
	private double prev_frame_counter;

	// States of the vision align subsystem
	public enum VisionAlignStates {
		sNotControlling(0), sAligning(1), sDelay(2), sConfirmTarget(3), sOnTarget(4);

		public final int value;

		private VisionAlignStates(int value) {
			this.value = value;
		}
	}

	// PID Gains
	Calibration angle_Kp = new Calibration("Alignment Angle Control Kp", 0.07, 0.0, 1.0);
	Calibration angle_Ki = new Calibration("Alignment Angle Control Ki", 0.04, 0.0, 1.0);
	Calibration angle_Kd = new Calibration("Alignment Angle Control Kd", 0.0, 0.0, 1.0);

	// Desired angle
	Calibration angleDesired = new Calibration("Desired Angle Alignment Offset", 0.0, -40.0, 40.0);

	private VisionAlignStates visionAlignState = VisionAlignStates.sNotControlling;
	private boolean visionAlignmentPossible = false;
	private boolean visionAlignmentOnTarget = false;
	private boolean visionAlignmentDesired = false;

	public static synchronized VisionAlignment getInstance() {
		if(visionAlignment == null)
			visionAlignment = new VisionAlignment();
		return visionAlignment;
	}

	/**
	 * Class to convert observed vision targets into drivetrain commands
	 * Said drivetrain commands will align the robot to the high goal.
	 */

	private VisionAlignment() {
		// Instantiate angle and distance PIDs
		anglePID = new VisionAlignAnglePID(angle_Kp.get(), angle_Ki.get(), angle_Kd.get());

		// Set max and min commands
		anglePID.setOutputRange(-0.75, 0.75);
		anglePID.setActualAsDerivTermSrc();
		anglePID.setintegratorDisableThresh(15.0); // Don't use I term until we're within 15 degrees

		// Make sure neither pid is running
		// CasserolePID is not running after construction

		// Make sure controller is off
		visionAlignmentOnTarget = false;

		gyroHistory = new InterpValueHistoryBuffer(50, 0);
		prev_frame_counter = 0;

		// Timer for sDelay state
		delayTimer = new Timer();
	}

	/**
	 * Periodic update function. call this during *_periodic() methods.
	 */
	public void GetAligned() {

		VisionProcessing vis = VisionProcessing.getInstance();

		// Is the control system capiable of performing alignment (vision and gyro are online)?
		visionAlignmentPossible = vis.isOnline() & Gyro.getInstance().isOnline();

		// Can we continue aligning (control system capable, and driver desires it)?
		boolean alignCanContinue = visionAlignmentDesired & visionAlignmentPossible;

		// Can we start aligning (continuing is possible, and we see a target at a reasonable location)?
		boolean alignCanStart = alignCanContinue & vis.getTarget().isTargetFound();

		// Save historical values
		// Tracking the historical values is needed to offset the delays in visino processing. From the time a frame
		// is captured by the camera, to when it goes over the network, gets processed by the coprocessor, put
		// on the network again, and finally qualified by the vision qualification system, there's a decent amount of delay.
		// Some of those processing times are measured, others are estimated and assumed fixed. The time all rolls up
		// to a single estimate of what time the image was captured at (relative to the getFPGATimestamp() timeframe.
		// We store a brief history of sensor readings (gyro or encoders). Every time we get a processed image frame,
		// we will look up what the gyro/distance readings were at the time the image was captured. We then use the image
		// processing data to define a new setpoint based on where image processing says we should have been. Finally,
		// this setpoint is given to the PID algorithms, which close-loop control the drivetrain around gyro and encoders,
		// moving it to setpoints determined by the vision processing system.
		// We effectively get a boost in bandwidth of our control algorithm.
		double timeNow = Timer.getFPGATimestamp();

		double delayTimeStart = 0;

		double angleOffset = 0;
		double distOffset = 0;
		double pidAngleErr = 0;
		double pidDistErr = 0;

		angleOffset = vis.getTarget().getTargetOffsetDegrees() - angleDesired.get();

		pidAngleErr = anglePID.getCurError();

		gyroHistory.insert(timeNow, Gyro.getInstance().getAngle());

		// If vision align is possible, and we have a new frame, and that frame has a target, update the setpoints.
		if((visionAlignmentPossible) &
				(prev_frame_counter != vis.getFrameCount()) &
				(vis.getTarget().isTargetFound()))
		{
			// update the gyro-based setpoints
			gyroAngleLastFrame = gyroHistory.getValAtTime(vis.getEstCaptureTime());
			gyroAngleDesiredLastFrame = gyroAngleLastFrame + (angleOffset);
			prev_frame_counter = vis.getFrameCount();
		}

		// Temp for tuning PID's (lock setpoints at zero, and tune PID's to see how fast you can correct to this setpoint)
		// RobotState.visionGyroAngleDesiredAtLastFrame = 0;
		// RobotState.visionDistanceDesiredAtLastFrame = 0;

		// Execute State Machine
		if(visionAlignState == VisionAlignStates.sOnTarget) {

			if(!(Math.abs(angleOffset) < angleTol + angleTolHyst)) { // If we get too far off target (based on camera input)...
				visionAlignmentOnTarget = false; // Set Off Target

				anglePID.setAngle(gyroAngleDesiredLastFrame); // "Take Pic"

				visionAlignState = VisionAlignStates.sAligning; // Change State
			}
			else if(!alignCanContinue) { // If we shouldn't continue vision alignment...
				visionAlignmentOnTarget = false;
				// Turn off pids
				anglePID.stop();
				// Change State
				visionAlignState = VisionAlignStates.sNotControlling;
			}
			else { // maintain state
				visionAlignState = VisionAlignStates.sOnTarget;
			}
		}
		else if(visionAlignState == VisionAlignStates.sAligning) {
			if(!alignCanContinue) { // If we shouldn't continue to attempt to align...
				visionAlignmentOnTarget = false;
				// Turn off pids
				anglePID.stop();
				// Change State
				visionAlignState = VisionAlignStates.sNotControlling;
			}
			else if((Math.abs(pidAngleErr) < angleTol))	{ // If we get within our tolerance (via closed loop)

				// Set delay start
				delayTimeStart = Timer.getFPGATimestamp();

				// Change State
				visionAlignState = VisionAlignStates.sDelay;
			}
			else { // maintain state
				visionAlignState = VisionAlignStates.sAligning;
			}
		}
		else if(visionAlignState == VisionAlignStates.sConfirmTarget) {
			if(!alignCanContinue) { // If we shouldn't continue to attempt to align...
				visionAlignmentOnTarget = false;
				// Turn off pids
				anglePID.stop();
				// Change State
				visionAlignState = VisionAlignStates.sNotControlling;

			}
			else if((Math.abs(angleOffset) < angleTol)) {
				// If we get within our tolerance (based on camera)
				// Set On Target
				visionAlignmentOnTarget = true;

				// Change State
				visionAlignState = VisionAlignStates.sOnTarget;

			}
			else {
				anglePID.setAngle(gyroAngleDesiredLastFrame); // "Take Pic"

				visionAlignState = VisionAlignStates.sAligning;
			}
		}
		else if(visionAlignState == VisionAlignStates.sDelay) {

			if((Timer.getFPGATimestamp() - delayTimeStart) * 1000 >= (RobotConstants.TOTAL_VISION_DELAY_S * 1000.0 * 1.25)) {
				// Change State
				visionAlignState = VisionAlignStates.sConfirmTarget;

			}
			else if(Math.abs(angleOffset) > angleTol) {
				visionAlignState = VisionAlignStates.sAligning;
			}
			else {
				// Maintain State
				visionAlignState = VisionAlignStates.sDelay;
			}

		}
		else { // visionAlignState == VisionAlignStates.sNotControlling
			visionAlignmentOnTarget = false;

			if(alignCanStart) { // If we should start attempting to vision track...
				// Reset integrators and start pids
				anglePID.start();

				// "Take Pic"
				anglePID.setAngle(gyroAngleDesiredLastFrame);

				// Change State
				visionAlignState = VisionAlignStates.sAligning;

			}
			else { // maintain state
				visionAlignState = VisionAlignStates.sNotControlling;
			}
		}
	}

	/**
	 * Update all calibrations. Should only be called if pid's are not in use
	 * soooo, during disabled periodic.
	 */
	public void updateGains() {
		if(angle_Kp.isChanged()) {
			anglePID.setKp(angle_Kp.get());
			angle_Kp.acknowledgeValUpdate();
		}

		if(angle_Ki.isChanged()) {
			anglePID.setKi(angle_Ki.get());
			angle_Ki.acknowledgeValUpdate();
		}

		if(angle_Kd.isChanged()) {
			anglePID.setKd(angle_Kd.get());
			angle_Kd.acknowledgeValUpdate();
		}
	}

	/**
	 * Getter setter stuff
	 */

	public double getVisionAlignState() {
		if(visionAlignState == VisionAlignStates.sOnTarget) {
			return 4.0;
		}
		else if(visionAlignState == VisionAlignStates.sConfirmTarget) {
			return 3.0;
		}
		else if(visionAlignState == VisionAlignStates.sDelay) {
			return 2.0;
		}
		else if(visionAlignState == VisionAlignStates.sAligning) {
			return 1.0;
		}
		else { // visionAlignState == VisionAlignStates.sNotControlling
			return 0.0;
		}
	}

	public String getVisionAlignStateName() {
		return visionAlignState.toString();
	}

	public boolean getVisionAlignmentPossible() {
		return visionAlignmentPossible;
	}

	public boolean getVisionAlignmentOnTarget() {
		return visionAlignmentOnTarget;
	}

	public void setVisionAlignmentDesired(boolean isDesired) {
		visionAlignmentDesired = isDesired;
	}

	public boolean getVisionAlignmentDesired() {
		return visionAlignmentDesired;
	}

	public double getGyroAngleDesiredAtLastFrame() {
		return gyroAngleDesiredLastFrame;
	}

	public double getGyroAngleAtLastFrame() {
		return gyroAngleLastFrame;
	}

	public double getRotateCmd() {
		return anglePID.getOutputCommand();
	}
}