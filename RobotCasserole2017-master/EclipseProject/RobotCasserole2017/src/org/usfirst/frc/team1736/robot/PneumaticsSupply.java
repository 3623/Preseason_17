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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;

public class PneumaticsSupply {
	private static PneumaticsSupply localPS = null;
	Compressor comp;

	private boolean enabled;

	private static AnalogInput storagePressureSensor;

	public static PneumaticsSupply getInstance() {
		if(localPS == null) {
			localPS = new PneumaticsSupply();
		}
		return localPS;
	}

	private PneumaticsSupply() {
		comp = new Compressor(RobotConstants.PCM_CAN_DEVICE_ID);
		comp.setClosedLoopControl(true); // ensure we are running by default
		enabled = true;

		storagePressureSensor = new AnalogInput(RobotConstants.AIR_PRESSURE_SENSOR_PORT);
	}

	/**
	 * 
	 * @return System unregulated pressure in psi
	 */
	public double getStoragePress() {
		return ((storagePressureSensor.getVoltage() / 5.0) - 0.1) * 150.0 / 0.8;
	}

	/**
	 * \
	 * 
	 * @return Compressor current draw in amps
	 */
	public double getCompressorCurrent() {
		return comp.getCompressorCurrent();
	}

	/**
	 * 
	 * @param state
	 *            pass True run the compressor up to top pressure, false to turn it off.
	 */
	public void setCompressorEnabled(boolean state) {
		comp.setClosedLoopControl(state);
		if(state == false) {
			comp.stop();
		}
		enabled = state;
	}

	/**
	 * 
	 * @return true if compressor is enabled to run, false if disabled.
	 */
	public boolean compressorIsEnabled() {
		return enabled;
	}
}
