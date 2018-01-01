package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
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
import edu.wpi.first.wpilibj.Spark;

public class ClimberControl {
	private static ClimberControl climberControl = null;

	private static boolean isClimbEnabled = true; // always enabled for now for testing

	private PowerDistributionPanel pdp_ref = null;

	private boolean climbCurrentTooHigh;
	private int currentTooHighDebounceLoopCount;

	// Declare Motor Control
	Spark climbMotor1 = new Spark(RobotConstants.CLIMBER_MOTOR1_PWM_PORT);
	Spark climbMotor2 = new Spark(RobotConstants.CLIMBER_MOTOR2_PWM_PORT);

	public static synchronized ClimberControl getInstance() {
		if(climberControl == null)
			climberControl = new ClimberControl();
		return climberControl;
	}

	/**
	 * Must be called before update. The parent class must indicate what PDP
	 * this climber control should use to evaluate motor current.
	 */
	public void setPDPReference(PowerDistributionPanel pdp_in) {
		pdp_ref = pdp_in;
	}

	private ClimberControl() {
		// Init Motor to off
		climbMotor1.set(0.0);
		climbMotor2.set(0.0);
	}

	// Climber Control
	public void update() {
		double operatorClimbCmd = OperatorController.getInstance().getClimbSpeedCmd();
		double climb_speed;

		evalCurrentDraw();

		// if we aren't enabled, or if we've got too much current, don't let the climb happen.
		if(climbCurrentTooHigh || !isClimbEnabled) {
			climb_speed = 0.0;
		}
		else {
			climb_speed = Math.abs(operatorClimbCmd); // Only allow climb in one direction (must be negative)
		}

		climbMotor1.set(climb_speed);
		climbMotor2.set(climb_speed);
	}

	public void setClimbEnabled(boolean isEnabled) {
		isClimbEnabled = isEnabled;
	}

	public boolean isClimbEnabled() {
		return isClimbEnabled;
	}

	public boolean isCurrentTooHigh() {
		return climbCurrentTooHigh;
	}

	/**
	 * Evaluates if we've exceeded our current draw capacity in either motor (in danger of burning out).
	 * If we have, set the disable flag to turn off motor output. Debounce current for some number of loops
	 * before resetting flag.
	 */
	private void evalCurrentDraw() {
		// Guard against potential develompent errors.
		if(pdp_ref == null) {
			System.out.println("WARNING - Software team has made a mistake! Tell them! They did not call setPDPReference from climber at the right time.");
			return;
		}

		if(Math.abs(pdp_ref.getCurrent(RobotConstants.CLIMBER_MOTOR1_PDP_CH)) > RobotConstants.CLIMBER_MOTOR_MAX_ALLOWABLE_CURRENT_A ||
				Math.abs(pdp_ref.getCurrent(RobotConstants.CLIMBER_MOTOR2_PDP_CH)) > RobotConstants.CLIMBER_MOTOR_MAX_ALLOWABLE_CURRENT_A) {
			climbCurrentTooHigh = true;
			currentTooHighDebounceLoopCount = 0;
		}
		else {
			if(currentTooHighDebounceLoopCount < RobotConstants.CLIMBER_MOTOR_EXCESS_CURRENT_DBNC_LOOPS) {
				currentTooHighDebounceLoopCount++;
			}
			else {
				climbCurrentTooHigh = false;
			}
		}
	}
}
