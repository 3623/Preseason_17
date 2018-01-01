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
import org.usfirst.frc.team1736.lib.CasserolePID.CasserolePID;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class DriveTrainWheelSpeedPI extends CasserolePID {
	SpeedController spdctrl;
	Encoder encoder;

	Calibration K_ff_cal;
	Calibration K_p_cal;
	Calibration K_i_cal;

	boolean enabled;
	boolean gyroCompEnabled;
	boolean gyroCompInverted;
	private final double gyroCompGain_cmdPerDegErr = 0.5 / 5.0;
	private double headingSetpoint;

	public DriveTrainWheelSpeedPI(SpeedController spdctrl_in, Encoder encoder_in, Calibration K_ff_cal_in, Calibration K_p_cal_in, Calibration K_i_cal_in) {
		super(K_p_cal_in.get(), K_i_cal_in.get(), 0, K_ff_cal_in.get(), 0, 0);
		this.threadName = "Drivetrain Velocity PID";

		spdctrl = spdctrl_in;
		encoder = encoder_in;
		K_ff_cal = K_ff_cal_in;
		K_p_cal = K_p_cal_in;
		K_i_cal = K_i_cal_in;

		// Set defaults
		enabled = false;
		headingSetpoint = 0;
		gyroCompEnabled = false;
		gyroCompInverted = false;

		// Motor Controllers allow a fixed 1/-1 range
		this.setOutputRange(-1, 1);
	}

	/**
	 * Update whether this PI controller should be enabled or not
	 * PID threads are started/stopped when enable changes state.
	 * Integrators are also reset each time the controller is reset.
	 * 
	 * @param enable_in
	 *            true to run this controller, false to disable it.
	 */
	public void setEnabled(boolean enable_in) {
		if(enable_in != enabled) {
			// Only bother to do something if we're changing the enabled state.
			if(enable_in == true & enabled == false) {
				// We used to be disabled, but now want to enable.
				this.resetIntegrators();
				this.start();
			} else if(enable_in == false & enabled == true) {
				this.stop();
			}
			enabled = enable_in;
		}
	}

	/**
	 * Update the calibrations for the PI(f) controller
	 */
	public void updateCal() {
		// Note acknowledgments must be done elsewhere
		// since there will me many classes which use
		// the same calibrations here.
		if(K_ff_cal.isChanged()) {
			this.setKf(K_ff_cal.get());
		}

		if(K_p_cal.isChanged()) {
			this.setKp(K_p_cal.get());
		}

		if(K_i_cal.isChanged()) {
			this.setKi(K_i_cal.get());
		}
	}

	public void setGyroCompEnabled(boolean en) {
		gyroCompEnabled = en;
	}

	public void setGyroCompInverted(boolean inv) {
		gyroCompInverted = inv;
	}

	public void setDesiredHeading(double hd) {
		headingSetpoint = hd;
	}

	public double getDesiredHeading() {
		return headingSetpoint;
	}

	@Override
	protected double returnPIDInput() {
		return encoder.getRate() * 60.0; // Assume motor controller scaled in rev/sec, return RPM
	}

	@Override
	protected void usePIDOutput(double pidOutput) {
		double gyroCompOutput;
		if(enabled) {
			// If we wanted gyro compensation and we have a gyro, do a very simple P controller
			// Allow for inversion of the gyro control effort since motors are flipped on each side of the drivetrain
			// (and this class takes care of all dt motors)
			if(gyroCompEnabled & Gyro.getInstance().isOnline()) {
				gyroCompOutput = -gyroCompGain_cmdPerDegErr * (headingSetpoint - Gyro.getInstance().getAngle());
				if(gyroCompInverted) {
					gyroCompOutput *= -1.0;
				}
			}
			else {
				// Don't use gyro compensation
				gyroCompOutput = 0;
			}

			spdctrl.set(Math.max(-1, Math.min(1, pidOutput + gyroCompOutput)));
		}
	}
}
