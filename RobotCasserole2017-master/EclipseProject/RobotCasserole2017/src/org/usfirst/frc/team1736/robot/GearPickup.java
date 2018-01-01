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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

public class GearPickup {
	private static GearPickup gearPickup = null;

	// Declare Motor Control
	Spark gearPickupMotor = new Spark(RobotConstants.INTAKE_MOTOR_PWM_PORT);

	// Declare Extender Control
	Solenoid intakeHPExtend = new Solenoid(RobotConstants.INTAKE_HP_EXTEND_SOLENOID_PORT);
	Solenoid intakeLPExtend = new Solenoid(RobotConstants.INTAKE_LP_EXTEND_SOLENOID_PORT);

	// Declaring Intake Calibration
	Calibration gearPickupSpeedCmd = new Calibration("Ground Pickup Gear Intake Motor Command", 1.0, 0.0, 1.0);

	// Intake Speed
	private final double pickupUp = 1.0;
	private static double pickupSpeedCommand = 0;

	OperatorController operatorControl;

	public static synchronized GearPickup getInstance() {
		if(gearPickup == null)
			gearPickup = new GearPickup();
		return gearPickup;
	}

	private GearPickup() {
		operatorControl = OperatorController.getInstance();

		// Init Motor to off
		gearPickupMotor.set(0.0);

		// Init position to up
	}

	public void update() {
		if(operatorControl.getPickupPosCmd() && operatorControl.getPickupSpeedCmd()) {
			// pickup gear - down and motors run fwd
			gearPickupMotor.set(gearPickupSpeedCmd.get());
			IntakeExtend();
		}
		else if(operatorControl.getPickupPosCmd() && !(operatorControl.getPickupSpeedCmd())) {
			// Gear placement - down and motor off
			gearPickupMotor.set(0.0);
			IntakeExtend();
		}
		else if(!(operatorControl.getPickupPosCmd()) && operatorControl.getPickupSpeedCmd()) {
			// eject gear - down and motor runs rev
			gearPickupMotor.set(-gearPickupSpeedCmd.get());
			IntakeExtend();
		}
		else {// !(operatorControl.getPickupPosCmd()) && !(operatorControl.getPickupSpeedCmd())
			// Not using - up and motor off
			gearPickupMotor.set(0.0);
			IntakeRetract();
		}
	}

	public void IntakeExtend() {
		intakeLPExtend.set(true);
	}

	public void IntakeRetract() {
		intakeLPExtend.set(false);
	}

	public boolean getPosCmd() {
		return operatorControl.getPickupPosCmd();
	}

	public boolean getSpeedCmd() {
		return operatorControl.getPickupSpeedCmd();
	}
}
