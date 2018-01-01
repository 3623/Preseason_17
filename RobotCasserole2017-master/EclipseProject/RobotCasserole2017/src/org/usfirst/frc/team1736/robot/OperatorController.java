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

import org.usfirst.frc.team1736.lib.HAL.Xbox360Controller;

public class OperatorController extends Xbox360Controller {
	private static OperatorController controller = null;

	// Operator shooter command interpretation variables
	boolean pev_State;

	public static synchronized OperatorController getInstance() {
		if(controller == null)
			controller = new OperatorController(1);
		return controller;
	}

	private OperatorController(int joystick_id) {
		super(joystick_id);
	}

	public boolean getGearSolenoidCmd() {
		return RStickButton();
	}

	public double getClimbSpeedCmd() {
		return LStick_Y();
	}

	public boolean getIntakeDesiredCmd() {
		return RB();
	}

	public boolean getEjectDesiredCmd() {
		return LB();
	}

	public boolean getPickupPosCmd() {
		return RB();
	}

	public boolean getPickupSpeedCmd() {
		return LB();
	}

	public boolean getHopperFwdOverride() {
		return StartButton();
	}

	public boolean getHopperRevOverride() {
		return BackButton();
	}

	public double getGearFlapCommand() {
		return LTrigger();
	}

	/**
	 * Do everything needed to update the operator control states.
	 * Must be called every loop.
	 */
	public void update() {
		boolean shootFlag;
		boolean rising_edge;
		boolean falling_edge;

		// Prep to Shoot or Disable shoot Commands
		if(A()) {
			ShotControl.getInstance().setDesiredShooterState(ShotControl.ShooterStates.PREP_TO_SHOOT);
		}

		if(B()) {
			ShotControl.getInstance().setDesiredShooterState(ShotControl.ShooterStates.NO_SHOOT);
		}

		// Shoot Command
		shootFlag = (RTrigger() > 0.5 || Y() == true);
		if(shootFlag & pev_State == false) {
			rising_edge = true;
		}
		else {
			rising_edge = false;
		}

		if(!shootFlag & pev_State == true) {
			falling_edge = true;
		}
		else {
			falling_edge = false;
		}

		if(rising_edge == true) {
			ShotControl.getInstance().setDesiredShooterState(ShotControl.ShooterStates.SHOOT);
		}
		else if(falling_edge == true) {
			ShotControl.getInstance().setDesiredShooterState(ShotControl.ShooterStates.PREP_TO_SHOOT);
		}

		pev_State = shootFlag;
		// end of shooter update code

		/* LED color Selections */
		if(DPadDown()) {
			LEDSequencer.getInstance().setNoneDesiredPattern();
		}
		else if(DPadUp()) {
			LEDSequencer.getInstance().setBothDesiredPattern();
		}
		else if(DPadLeft()) {
			LEDSequencer.getInstance().setGearDesiredPattern();
		}
		else if(DPadRight()) {
			LEDSequencer.getInstance().setFuelDesiredPattern();
		}
	}
}
