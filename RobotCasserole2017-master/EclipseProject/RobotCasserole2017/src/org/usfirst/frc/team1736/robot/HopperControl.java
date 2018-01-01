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

import edu.wpi.first.wpilibj.Spark;

public class HopperControl {
	private static HopperControl hopperControl = null;

	public enum HopperStates {
		HOPPER_OFF, HOPPER_FWD, HOPPER_REV;
	}

	// Declare Motor Control
	private Spark hopMotor = new Spark(RobotConstants.HOPPER_MOTOR_PWM_PORT);

	// Declaring Hopper Calibration
	Calibration hopperMotorCmd = new Calibration("Hopper Feed Motor Command", 0.7, 0.0, 1.0);

	private double motorCmd = 0;

	// Hopper Speed
	private double hopSpeedOff = 0.0;

	public static synchronized HopperControl getInstance() {
		if(hopperControl == null)
			hopperControl = new HopperControl();
		return hopperControl;
	}

	private HopperControl() {
		// Init Motor to off
		hopMotor.set(0.0);
	}

	public void update() {
		if(OperatorController.getInstance().getHopperFwdOverride())
			motorCmd = -1 * hopperMotorCmd.get();
		else if(OperatorController.getInstance().getHopperRevOverride())
			motorCmd = 1 * hopperMotorCmd.get();
		else if(ShotControl.getInstance().getHopperFeedCmd() == HopperStates.HOPPER_FWD)
			motorCmd = -1 * hopperMotorCmd.get();
		else if(ShotControl.getInstance().getHopperFeedCmd() == HopperStates.HOPPER_REV)
			motorCmd = 1 * hopperMotorCmd.get();
		else {
			motorCmd = hopSpeedOff;
		}
		hopMotor.set(motorCmd);
	}

	public double getHopperMotorCmd() {
		return motorCmd;
	}
}
