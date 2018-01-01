package org.usfirst.frc.team1736.robot.auto;

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoEvent;
import org.usfirst.frc.team1736.lib.FalconPathPlanner.PathPlannerAutoEvent;
import org.usfirst.frc.team1736.robot.DriveTrain;
import org.usfirst.frc.team1736.robot.FlappyGear;
import org.usfirst.frc.team1736.robot.RobotConstants;

public class AutoEventGetToDaHoppaRight extends AutoEvent {
	PathPlannerAutoEvent driveEvent;

	// These waypoints will probably need to be changed a lot if the PIDs are retuned
	private static final double[][] waypoints = new double[][] {
		{0, 0, 0},
		{0, -5.5, 0},
		{-8.5, -5.5, 0}
	};

	private static final double time = 3;

	public AutoEventGetToDaHoppaRight() {
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
	}

	@Override
	public void userStart() {
		FlappyGear.getInstance().setAngle(RobotConstants.FLAP_DOWN_DEG);
		driveEvent.userStart();
	}

}
