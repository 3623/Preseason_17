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

public class FlappyGear {
	private static FlappyGear flappyGear = null;

	int FLAP_DOWN_DEG = 190;
	int FLAP_UP_DEG = 65;
	Servo servo;

	public static synchronized FlappyGear getInstance() {
		if(flappyGear == null)
			flappyGear = new FlappyGear();
		return flappyGear;
	}

	private FlappyGear() {
		servo = new Servo(RobotConstants.GEAR_FLAP_SERVO_PWM_PORT);
	}

	public void update() {

		servo.setAngle((FLAP_DOWN_DEG - FLAP_UP_DEG) * OperatorController.getInstance().getGearFlapCommand() + FLAP_UP_DEG);
	}

	public void setAngle(int angle) {
		servo.setAngle(angle);
	}
}
