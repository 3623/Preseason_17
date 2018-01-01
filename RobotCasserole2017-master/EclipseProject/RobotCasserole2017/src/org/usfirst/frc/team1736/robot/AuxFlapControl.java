package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class AuxFlapControl {
	private static AuxFlapControl auxflap = null;

	// Declare Extender Control
	Solenoid auxFlapSol = new Solenoid(RobotConstants.AUX_FLAP_SOLENOID_PORT);

	// Wait time
	double SOLENOID_HOLD_TIME = 2.0;
	double time_start;

	// Simple state
	boolean cycle_done = false;

	// Solenoid command
	boolean solenoid_cmd = false;

	public static synchronized AuxFlapControl getInstance() {
		if(auxflap == null)
			auxflap = new AuxFlapControl();
		return auxflap;
	}

	private AuxFlapControl() {
		cycle_done = false;
		solenoid_cmd = false;
		auxFlapSol.set(solenoid_cmd);
	}

	public void update() {
		if(cycle_done) {
			solenoid_cmd = false;
		}
		else if(Timer.getFPGATimestamp() - time_start > SOLENOID_HOLD_TIME) {
			cycle_done = true;
			solenoid_cmd = false;
		}
		else {
			solenoid_cmd = true;
		}
		auxFlapSol.set(solenoid_cmd);
	}

	public void startCycle() {
		solenoid_cmd = true;
		auxFlapSol.set(solenoid_cmd);
		time_start = Timer.getFPGATimestamp();
	}

	public void endCycle() {
		cycle_done = false;
		solenoid_cmd = false;
		auxFlapSol.set(solenoid_cmd);
	}

	public boolean getAuxFlapCmd() {
		return solenoid_cmd;
	}
}
