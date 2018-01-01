package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;

/**
 * Path from starting point to center lift. We go sideways because that's how the robot is.
 *
 */
public class AutoEventDriveToCenterLift extends AutoEvent {
	PathPlannerAutoEvent driveSideways;

	private static final double[][] waypoints = new double[][] {
		{0, 0, 0},
		{8.5, -0.3, 0}
	};

	private static final double time = 2.25;

	public AutoEventDriveToCenterLift() {
		driveSideways = new PathPlannerAutoEvent(waypoints, time,
				DriveTrain.getInstance().getFrontLeftCTRL(), DriveTrain.getInstance().getFrontRightCTRL(),
				DriveTrain.getInstance().getRearLeftCTRL(), DriveTrain.getInstance().getRearRightCTRL());
	}

	public void userForceStop() {
		driveSideways.userForceStop();
	}

	public boolean isTriggered() {
		return driveSideways.isTriggered();
	}

	public boolean isDone() {
		return driveSideways.isDone();
	}

	@Override
	public void userUpdate() {
		driveSideways.userUpdate();
	}

	@Override
	public void userStart() {
		driveSideways.userStart();
	}

}
