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

import org.usfirst.frc.team1736.lib.Sensors.ADXRS453_Gyro;

/**
 * This class is intended to just be a simple singleton wrapper around our gyro. It can implement methods specific to our gyro (such as inverting the angle)
 *
 */
public class Gyro {
	private static Gyro gyro;
	private static ADXRS453_Gyro adxrs453;
	private static int angleOffset;

	public static synchronized Gyro getInstance() {
		if(gyro == null)
			gyro = new Gyro();
		return gyro;
	}

	private Gyro() {
		adxrs453 = new ADXRS453_Gyro();
		angleOffset = 0;
	}

	public double getAngle() {
		return angleOffset - adxrs453.getAngle();
	}

	public void reset() {
		adxrs453.reset();
	}

	public void setAngleOffset(int angle) {
		angleOffset = angle;
	}

	public int getAngleOffset() {
		return angleOffset;
	}

	public boolean isOnline() {
		// return adxrs453.isOnline();
		return true; // Temp, for bench debugging
	}
}
