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

import org.usfirst.frc.team1736.lib.SignalMath.AveragingFilter;
import org.usfirst.frc.team1736.lib.SignalMath.DerivativeCalculator;

public class ShotCounter {
	private static ShotCounter shotCounter;
	private DerivativeCalculator IDotCalc;
	private DerivativeCalculator IDoubleDotCalc;
	private AveragingFilter IDoubleDotFilter;
	private final double IDoubleDotThresh = -4000; // Amps per sec per sec
	private int currCount;
	private boolean aboveThresh;

	public static synchronized ShotCounter getInstance() {
		if(shotCounter == null)
			shotCounter = new ShotCounter();
		return shotCounter;
	}

	/**
	 * Simple class to attempt to count the number of balls we've shot.
	 * Mostly just for driver feedback now. Looks for spikes in the shooter motor current draw while
	 * we are attempting to shoot.
	 */
	private ShotCounter() {
		IDotCalc = new DerivativeCalculator();
		IDoubleDotCalc = new DerivativeCalculator();
		IDoubleDotFilter = new AveragingFilter(5, 0);
		currCount = 0;
		aboveThresh = false;
	}

	public void update() {
		// See the offboard calculator matlab scripts for more info on this algorithm.
		// Through experimentation with data we recorded on the robot, we determined a good algorithm
		// for counting shots was:
		// --Calc the second derivative of current draw (I double dot)
		// --Filter with a 5-pt sliding average window
		// --Look for negative peaks of I double-dot, below -4000 amps/sec/sec
		// --Every time I double dot crosses that threshold, increment the shot count.
		double rpm = ShooterWheelCtrl.getInstance().getShooterActualVelocityRPM();
		double shooterCurrent = ShooterWheelCtrl.getInstance().getOutputCurrent();
		if(rpm > ShooterWheelCtrl.getInstance().getShooterDesiredRPM() * 0.5) { // Only run logic if shooter wheel is above half speed
			double IDot = IDotCalc.calcDeriv(shooterCurrent);
			double IDoubleDot = IDoubleDotCalc.calcDeriv(IDot);
			double IDoubleDotFiltered = IDoubleDotFilter.filter(IDoubleDot);
			if(IDoubleDotFiltered < IDoubleDotThresh && aboveThresh == false) {
				aboveThresh = true;
			}
			if(IDoubleDotFiltered >= IDoubleDotThresh && aboveThresh == true) {
				currCount = currCount + 1;
				aboveThresh = false;
			}
		}
	}

	public int getCurrCount() {
		return currCount;
	}

	public double getCurrCountLog() {
		return currCount;
	}
}
