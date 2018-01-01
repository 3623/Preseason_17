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

import edu.wpi.first.wpilibj.Servo;

/**
 * @author gerthcm
 *
 */
public class CameraServoMount {
	public enum CamPos {
		SHOOT, GEAR, INTAKE
	}

	// State variables
	public double cur_pan_angle;
	public double cur_tilt_angle;
	public CamPos curCamPos;

	// Startup conditions
	private static final CamPos startupPos = CamPos.SHOOT;

	// Servo objects for mount servos
	private Servo pan_servo;
	private Servo tilt_servo;

	/**
	 * Constructor - initializes all the objects for a camera servo mount. Takes nothing, returns nothing.
	 */
	CameraServoMount() {
		pan_servo = new Servo(RobotConstants.CAMERA_PAN_SERVO_PWM_PORT);
		tilt_servo = new Servo(RobotConstants.CAMERA_TILT_SERVO_PWM_PORT);
		setCameraPos(startupPos);
	}

	public void update() {
		if(DriverController.getInstance().getGearCamAlign()) {
			setCameraPos(CamPos.GEAR);
		}
		else if(DriverController.getInstance().getIntakeCamAlign()) {
			setCameraPos(CamPos.INTAKE);
		}
		else if(DriverController.getInstance().getShooterCamAlign()) {
			setCameraPos(CamPos.SHOOT);
		}
	}

	/**
	 * Commands the servos to the right spots based on the value of camera position in
	 * 
	 * @param in
	 */
	public void setCameraPos(CamPos in) {
		resolveCamPos(in);
		pan_servo.setAngle(cur_pan_angle);
		tilt_servo.setAngle(cur_tilt_angle);
	}

	/**
	 * Sets the pan and tilt internal variables per the
	 * position specified in the input argument.
	 * 
	 * @param in
	 *            - position to set the camera to.
	 */
	private void resolveCamPos(CamPos in) {
		curCamPos = in;

		switch(in) {
		case INTAKE:
			cur_pan_angle = RobotConstants.INTAKE_PAN_ANGLE;
			cur_tilt_angle = RobotConstants.INTAKE_TILT_ANGLE;
			break;
		case SHOOT:
			cur_pan_angle = RobotConstants.SHOOT_PAN_ANGLE;
			cur_tilt_angle = RobotConstants.SHOOT_TILT_ANGLE;
			break;
		case GEAR:
			cur_pan_angle = RobotConstants.GEAR_PAN_ANGLE;
			cur_tilt_angle = RobotConstants.GEAR_TILT_ANGLE;
			break;
		default:
			System.out.println("Warning - commanded camera position " + in.name() + " is not recognized!");
			break;
		}
	}
}
