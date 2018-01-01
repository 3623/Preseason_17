package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.robot.GearControl;

import edu.wpi.first.wpilibj.Timer;

/**
 * Releases gear. That is all.
 * GEEEEAAARRRRR!
 *
 */
public class AutoEventOpenGearMechanism extends AutoEvent {
	Timer timer;

	public AutoEventOpenGearMechanism() {
		timer = new Timer();
	}

	@Override
	public void userUpdate() {

	}

	@Override
	public void userForceStop() {

	}

	@Override
	public boolean isTriggered() {
		return false;
	}

	@Override
	public boolean isDone() {
		if(timer.get() > .5)
			return true;
		return false;
	}

	@Override
	public void userStart() {
		timer.start();
		GearControl.getInstance().openGearSolenoid();
	}

}
