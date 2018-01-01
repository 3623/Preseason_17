package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;
import org.usfirst.frc.team1736.robot.FlappyGear;
import org.usfirst.frc.team1736.robot.RobotConstants;
import org.usfirst.frc.team1736.robot.ShotControl;
import org.usfirst.frc.team1736.robot.ShotControl.ShooterStates;

public class AutoEventCatchHopper extends AutoEvent {
	PathPlannerAutoEvent driveEvent;

	private static final double[][] waypoints = new double[][] {
		{0, 0, 0},
		{0, -3.25, 0}
	};

	private static final double time = 1;

	public AutoEventCatchHopper() {
		driveEvent = new PathPlannerAutoEvent(waypoints, time,
				DriveTrain.getInstance().getFrontLeftCTRL(), DriveTrain.getInstance().getFrontRightCTRL(),
				DriveTrain.getInstance().getRearLeftCTRL(), DriveTrain.getInstance().getRearRightCTRL());
	}

	public void userForceStop() {
		driveEvent.userForceStop();
	}

	public boolean isTriggered() {
		return driveEvent.isTriggered();
	}

	public boolean isDone() {
		return driveEvent.isDone();
	}

	@Override
	public void userUpdate() {
		driveEvent.userUpdate();
		ShotControl.getInstance().setDesiredShooterState(ShooterStates.PREP_TO_SHOOT);
	}

	@Override
	public void userStart() {
		FlappyGear.getInstance().setAngle(RobotConstants.FLAP_DOWN_DEG);
		driveEvent.userStart();
	}

}
