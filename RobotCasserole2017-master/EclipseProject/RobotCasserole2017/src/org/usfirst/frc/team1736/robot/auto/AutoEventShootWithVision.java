package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.robot.ShotControl;
import org.usfirst.frc.team1736.robot.ShotControl.ShooterStates;
import org.usfirst.frc.team1736.robot.VisionAlignment;

/**
 * Auto event to run both vision alignment and shooter.
 * It would be expected that the target is already in view. If that is the case,
 * This will spool up the shooter and move the drivetrain to align. Once the drivetrain is aligned,
 * it will begin to feed the fuel into the shooter and will continue to do so until the robot is
 * disabled.
 *
 */
public class AutoEventShootWithVision extends AutoEvent {

	// Vision Alignment Control
	VisionAlignment visionAlignCTRL;

	// Shooter control
	ShotControl shotCTRL;

	@Override
	public void userUpdate() {
		visionAlignCTRL = VisionAlignment.getInstance();
		visionAlignCTRL.setVisionAlignmentDesired(true);
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.SHOOT);
	}

	@Override
	public void userForceStop() {
		visionAlignCTRL = VisionAlignment.getInstance();
		visionAlignCTRL.setVisionAlignmentDesired(false);
		shotCTRL = ShotControl.getInstance();
		shotCTRL.setDesiredShooterState(ShooterStates.NO_SHOOT);
	}

	@Override
	public boolean isTriggered() {
		return false;
	}

	@Override
	public boolean isDone() {
		return false;
	}

	@Override
	public void userStart() {

	}
}
