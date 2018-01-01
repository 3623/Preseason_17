package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.Calibration;

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

import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;

public class DriverController extends Xbox360Controller {
	private static DriverController controller = null;
	private boolean airCompState = true;

	public static synchronized DriverController getInstance() {
		if(controller == null)
			controller = new DriverController(0);
		return controller;
	}

	private DriverController(int joystick_id) {
		super(joystick_id);
	}

	public double getFwdRevCmd() {
		if(getDriveGearPlaceDriveMode()) {
			return expScaleCommand(-getXYRoundedToCaridinalPoints('X')) * 0.5;
		}
		else {
			return expScaleCommand(LStick_Y());
		}
	}

	public double getStrafeCmd() {
		if(getDriveGearPlaceDriveMode()) {
			return expScaleCommand(getXYRoundedToCaridinalPoints('Y'));
		}
		else {
			return expScaleCommand(LStick_X());
		}
	}

	public double getRotateCmd() {
		if(getDriveGearPlaceDriveMode()) {
			return expScaleCommand(RStick_X()) * 0.5;
		}
		else {
			return expScaleCommand(RStick_X());
		}
	}

	public boolean getGearCamAlign() {
		//return B();
		return false;//Unused
	}

	public boolean getIntakeCamAlign() {
		//return X();
		return false;//Unused
	}

	public boolean getShooterCamAlign() {
		//return Y();
		return false; //Unused
	}
	
	public boolean getTestClosedLoopModeEnabled(){
		return B();
	}
	
	public boolean getTestClosedLoopFwdDesired(){
		return Y();
	}
	
	public boolean getTestClosedLoopRevDesired(){
		return X();
	}

	public boolean getTestClosedLoopStrafeDesired(){
		return A();
	}
	public boolean getGyroReset() {
		return DPadUp();
	}

	public boolean getGyroReset90() {
		return DPadRight();
	}

	public boolean getGyroReset180() {
		return DPadDown();
	}

	public boolean getGyroReset270() {
		return DPadLeft();
	}

	public void updateAirCompEnabled() {
		if(StartButton())
			airCompState = true;
		if(BackButton())
			airCompState = false;
	}

	public boolean getAirCompEnableCmd() {
		return airCompState;
	}

	public boolean getAlignDesired() {
		return RB();
	}

	public boolean getDriveGearPlaceDriveMode() {
		return LB();
	}

	public void update() {
		updateAirCompEnabled();
		PneumaticsSupply.getInstance().setCompressorEnabled(getAirCompEnableCmd());

		if(getDriveGearPlaceDriveMode()) {
			VisionDelayCalibration.getInstance().setLEDRingActive(false);
		}
		else {
			VisionDelayCalibration.getInstance().setLEDRingActive(true);
		}

		// Update Gyro angle
		int angle = Gyro.getInstance().getAngleOffset();
		if(getGyroReset()) {
			angle = 0;
		}
		else if(getGyroReset90()) {
			angle = 90;
		}
		else if(getGyroReset180()) {
			angle = 180;
		}
		else if(getGyroReset270()) {
			angle = 270;
		}

		/*
		 * Alternate gyro angle update key idea
		 * If decide to try comment out if else above
		 * And update DriverController
		 * 
		 * if(DPadUp())
		 * {
		 * angle += 90;
		 * if (angle >= 360){
		 * angle = 0;
		 * }
		 * } else if(DPadDown())
		 * {
		 * angle -= 90;
		 * if (angle < 0){
		 * angle = 270;
		 * }
		 * }
		 */
		Gyro.getInstance().setAngleOffset(angle);
	}

	private double expScaleCommand(double input) {
		boolean isPos;

		// remember sign
		if(input > 0) {
			isPos = true;
		}
		else {
			isPos = false;
		}

		// Scale joystick by power
		double doug = Math.pow(Math.abs(input), 3);

		// re-invert if needed
		if(isPos == false) {
			doug = -doug;
		}

		return doug;
	}

	private double getXYRoundedToCaridinalPoints(char dir) {
		double theta = Math.atan2(LStick_Y(), LStick_X());

		// Cardinal points in radians
		double east = 0;
		double north = Math.PI / 2;
		double west = Math.PI;
		double south = -north;
		double ErrorMargin = Math.PI / 8;

		if((theta < ErrorMargin && theta > -ErrorMargin) || (theta > west - ErrorMargin && theta <= west) || (theta < -west + ErrorMargin && theta >= -west))
		{
			// Go east or west
			if(dir == 'X') {
				return LStick_X();
			} 
			else {
				return 0;
			}
		}
		else if((theta < north + ErrorMargin && theta > north - ErrorMargin) || (theta < south + ErrorMargin && theta > south - ErrorMargin)) {
			// go north or south
			if(dir == 'Y') {
				return LStick_Y();
			}
			else {
				return 0;
			}
		}
		else {
			return 0;
		}
	}
}
