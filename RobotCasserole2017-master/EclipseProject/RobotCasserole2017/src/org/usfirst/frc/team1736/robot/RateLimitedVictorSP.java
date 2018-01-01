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

import edu.wpi.first.wpilibj.VictorSP;

/**
 * Wrapper for a victorSP to make it very simple to rate-limit the output.
 * We thought this would be easier than our current limiting algorithm.
 * It was easier, but we still drained our battery like bonkers.
 * 
 * 200degF CIM motors are not happy CIM motors.
 * 
 * Do as we say, not as we do.
 *
 */

public class RateLimitedVictorSP extends VictorSP {
	double prevSetpoint = 0;

	public final double PER_LOOP_SETPOINT_RATE_LIMIT = 0.2;

	public RateLimitedVictorSP(int channel) {
		super(channel);
	}

	@Override
	public void set(double val) {
		double delta = val - prevSetpoint;

		if(delta > PER_LOOP_SETPOINT_RATE_LIMIT) {
			super.set(prevSetpoint + PER_LOOP_SETPOINT_RATE_LIMIT);
		}
		else if(delta < -PER_LOOP_SETPOINT_RATE_LIMIT) {
			super.set(prevSetpoint - PER_LOOP_SETPOINT_RATE_LIMIT);
		}
		else {
			super.set(val);
		}

		prevSetpoint = val;
	}
}
