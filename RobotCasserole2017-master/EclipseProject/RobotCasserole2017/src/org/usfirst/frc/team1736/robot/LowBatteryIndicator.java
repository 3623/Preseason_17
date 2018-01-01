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

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class LowBatteryIndicator {
	private static LowBatteryIndicator batteryIndicator = null;
	private boolean lowVoltageTriggered = false;
	private PowerDistributionPanel pdp = null;
	private AveragingFilter voltageFilter = null;
	private final double MIN_AVG_VOLTAGE = 8.0;
	private final double MIN_INSTANT_VOLTAGE = 6.0;

	public static LowBatteryIndicator getInstance() {
		if(batteryIndicator == null)
			batteryIndicator = new LowBatteryIndicator();
		return batteryIndicator;
	}

	private LowBatteryIndicator() {
		voltageFilter = new AveragingFilter(7, 12);
	}

	public void update() {
		if(pdp == null)
			return;
		double avgVoltage = voltageFilter.filter(pdp.getVoltage());
		if(avgVoltage < MIN_AVG_VOLTAGE || pdp.getVoltage() < MIN_INSTANT_VOLTAGE)
			lowVoltageTriggered = true;
	}

	public void setPDPReference(PowerDistributionPanel pdp) {
		this.pdp = pdp;
	}

	public boolean isBatteryDead() {
		return lowVoltageTriggered;
	}
}
