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

import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;

public class VisionAlignAnglePID extends CasserolePID {
	private double outputCmd = 0;

	/**
	 * Wrapper for PID algorithm to align the robot's drivetrain angle toward the high goal target
	 */
	VisionAlignAnglePID(double Kp_in, double Ki_in, double Kd_in) {
		super(Kp_in, Ki_in, Kd_in);
		this.threadName = "Vision Angle Alignment PID";
	}

	public void setAngle(double angle) {
		setSetpoint(angle);
	}

	@Override
	protected double returnPIDInput() {
		return Gyro.getInstance().getAngle();
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		// Limit to half range to reduce overshoot
		outputCmd = pidOutput;
	}

	@Override
	public void stop() {
		super.stop();
		outputCmd = 0;
	}

	public double getOutputCommand() {
		return outputCmd;
	}
}
