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

public class ShotControl {
	private static ShotControl shotControl = null;
	Calibration wheel_Set_Point_rpm;
	private HopperControl.HopperStates hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
	private ShooterWheelCtrl wheelCtrl;
	private ShooterStates desiredShooterState = ShooterStates.NO_SHOOT;

	public enum ShooterStates {
		NO_SHOOT, PREP_TO_SHOOT, SHOOT;
	}

	public static synchronized ShotControl getInstance() {
		if(shotControl == null)
			shotControl = new ShotControl();
		return shotControl;
	}

	/**
	 * Class to coordinate the shooter flywheel and hopper feeder actions
	 * We need to respond to driver commands, but never dump balls into the
	 * shooter if the RPM isn't high enough.
	 */
	private ShotControl() {
		wheel_Set_Point_rpm = new Calibration("Shooter Wheel Setpoint RPM", 3650, 0, 5000);
		hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
		wheelCtrl = ShooterWheelCtrl.getInstance();
		wheelCtrl.setShooterDesiredRPM(0);
	}

	public void update() {
		if(ShooterStates.NO_SHOOT == desiredShooterState) {
			// Operator requests everything turned off.
			hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
			wheelCtrl.setShooterDesiredRPM(0);
		}
		else if(ShooterStates.PREP_TO_SHOOT == desiredShooterState) {
			// Operator wants to prepare to shoot. Today, this means spooling up the shooter wheel.
			hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
			wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
		}
		else if((ShooterStates.SHOOT == desiredShooterState) & wheelCtrl.getShooterVelocityOK()) {
			// Operator wants to take the shot, and the shooter RPM is up to speed
			if(VisionAlignment.getInstance().getVisionAlignmentDesired()) {
				// Driver has robot under automatic (vision-assist) alignment.
				if(VisionAlignment.getInstance().getVisionAlignmentPossible() & VisionAlignment.getInstance().getVisionAlignmentOnTarget()) {
					// Vision alignment reports we are on target. Take the shot.
					hopperFeedCmd = HopperControl.HopperStates.HOPPER_FWD;
					wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
				}
				else {
					// Inhibit shot until vision alignment is achieved
					hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
					wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
				}
			}
			else {
				// Shooter is under manual alignment, just take the shot if RPM is ok
				hopperFeedCmd = HopperControl.HopperStates.HOPPER_FWD;
				wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
			}
		}
		else
		{ // Shot desired but wheel RPM is not OK
			// Just spool the wheel back up. Hopefully we get it fast enough to take a shot soon.
			hopperFeedCmd = HopperControl.HopperStates.HOPPER_OFF;
			wheelCtrl.setShooterDesiredRPM(wheel_Set_Point_rpm.get());
		}
	}

	public HopperControl.HopperStates getHopperFeedCmd() {
		return hopperFeedCmd;
	}

	public ShooterStates getDesiredShooterState() {
		return desiredShooterState;
	}

	public double getDesiredShooterStateOrdinal() {
		return desiredShooterState.ordinal();
	}

	public void setDesiredShooterState(ShooterStates state) {
		desiredShooterState = state;
	}
}
