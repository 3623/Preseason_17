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
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebPlots;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;

public class ShooterWheelCtrl {
	private static ShooterWheelCtrl wheelCtrl = null;

	Calibration Shooter_ff_Gain;
	Calibration Shooter_P_Gain;
	Calibration Shooter_I_Gain;
	Calibration Shooter_D_Gain;
	Calibration ErrorRange;
	private CANTalon shooterTalon;

	private double desiredVelocity = 0;
	private double actualVelocity = 0;
	private double motorVoltage = 0;
	private boolean isVelocityOk = false;

	double prevVel = 0;

	int velocityOkDbncTimer = 0;

	final int VELOCITY_RECOVERY_DBNC_LOOPS = 4;

	public static synchronized ShooterWheelCtrl getInstance() {
		if(wheelCtrl == null)
			wheelCtrl = new ShooterWheelCtrl();
		return wheelCtrl;
	}

	/**
	 * Configuration and control for the Talon SRX controlling the high-goal fuel shooter flywheel
	 */
	private ShooterWheelCtrl() {
		shooterTalon = new CANTalon(RobotConstants.SHOOTER_CAN_TALON_DEVICE_ID);
		Shooter_ff_Gain = new Calibration("Shooter FeedFwd Gain", 0.0269);
		Shooter_P_Gain = new Calibration("Shooter P Gain", 0.6);
		Shooter_I_Gain = new Calibration("Shooter I Gain", 0.01);
		Shooter_D_Gain = new Calibration("Shooter D Gain", 0.2);
		ErrorRange = new Calibration("Shooter Error Limit RPM", 100, 10, 1000);

		shooterTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); // Tells the SRX an encoder is attached to its input.
		shooterTalon.setProfile(0); // Select slot 0 for PID gains
		shooterTalon.changeControlMode(TalonControlMode.Speed); // Set that a PID algorithm should be used to control the output

		shooterTalon.setF(Shooter_ff_Gain.get()); // Set the PID algorithm gains based on calibration values
		shooterTalon.setP(Shooter_P_Gain.get());
		shooterTalon.setI(Shooter_I_Gain.get());
		shooterTalon.setD(Shooter_D_Gain.get());

		shooterTalon.enableBrakeMode(true); // Brake when 0 commanded (to stop shooter as fast as possible)
		shooterTalon.reverseOutput(false);
		shooterTalon.reverseSensor(true);
		shooterTalon.setIZone(100);
		
		CasseroleWebPlots.addNewSignal("Shooter Desired Speed", "RPM");
		CasseroleWebPlots.addNewSignal("Shooter Actual Speed", "RPM");
		CasseroleWebPlots.addNewSignal("Shooter Actual Current", "A");
	}

	/**
	 * Update the gains if needed
	 */

	public void updateGains() {
		// Set the PID algorithm gains based on calibration values
		if(Shooter_ff_Gain.isChanged()) {
			shooterTalon.setF(Shooter_ff_Gain.get());
			Shooter_ff_Gain.acknowledgeValUpdate();
		}

		if(Shooter_P_Gain.isChanged()) {
			shooterTalon.setP(Shooter_P_Gain.get());
			Shooter_P_Gain.acknowledgeValUpdate();
		}

		if(Shooter_I_Gain.isChanged()) {
			shooterTalon.setI(Shooter_I_Gain.get());
			Shooter_I_Gain.acknowledgeValUpdate();
		}

		if(Shooter_D_Gain.isChanged()) {
			shooterTalon.setD(Shooter_D_Gain.get());
			Shooter_D_Gain.acknowledgeValUpdate();
		}
	}

	/**
	 * Periodic update. Call during the *_update() methods.
	 * Sends the shooter RPM commands to the talon SRX while shooting, or forces it to zero output while not shooting.
	 * Leaving the PID running all the time caused it to dance back and forth and look like it was broken. Aside from being
	 * bad for the motors, everyone kept asking why the software was broken. So we fixed it.
	 * 
	 */
	public void update() {
		if(prevVel <= 0 && desiredVelocity > 0) {
			shooterTalon.changeControlMode(TalonControlMode.Speed);
		}
		else if(prevVel > 0 && desiredVelocity <= 0) {
			shooterTalon.changeControlMode(TalonControlMode.Voltage);
		}

		if(desiredVelocity > 0) {
			shooterTalon.setSetpoint(desiredVelocity);// set what speed the wheel should be running at
		}
		else {
			shooterTalon.set(0);// set what speed the wheel should be running at
		}

		actualVelocity = shooterTalon.getSpeed();
		motorVoltage = shooterTalon.getOutputVoltage();
		double Error = Math.abs(desiredVelocity - actualVelocity);

		// Debounce the recovery of shooter velocity.
		// As soon as the error gets too big, declare the velocity out of range
		// But after it comes back in range, wait some number of loops before calling the velocity good again
		if(Error > ErrorRange.get()) { // Robust controls == Threshold and debounce.
			isVelocityOk = false;
			velocityOkDbncTimer = 0;
		}
		else if(velocityOkDbncTimer < VELOCITY_RECOVERY_DBNC_LOOPS) {
			velocityOkDbncTimer++;
		}

		if(velocityOkDbncTimer >= VELOCITY_RECOVERY_DBNC_LOOPS) {
			isVelocityOk = true;
		}

		// Update desired velocity previous state
		prevVel = desiredVelocity;
		
		//Update real-time plot window
		double time = Timer.getFPGATimestamp();
		CasseroleWebPlots.addSample("Shooter Desired Speed", time, getShooterDesiredRPM());
		CasseroleWebPlots.addSample("Shooter Actual Speed", time, getShooterActualVelocityRPM());
		CasseroleWebPlots.addSample("Shooter Actual Current", time, getOutputCurrent());
	}

	public void setShooterDesiredRPM(double rpm) {
		desiredVelocity = rpm;
	}

	public double getShooterDesiredRPM() {
		return desiredVelocity;
	}

	public boolean getShooterVelocityOK() {
		return isVelocityOk;
	}

	public double getShooterActualVelocityRPM() {
		return actualVelocity;
	}

	public double getShooterMotorVoltage() {
		return motorVoltage;
	}

	public double getOutputCurrent() {
		return shooterTalon.getOutputCurrent();
	}
}