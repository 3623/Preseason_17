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

import org.usfirst.frc.team1736.lib.AutoSequencer.AutoSequencer;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayFromLiftNoCross;
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayLeftFromLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventBackAwayRightFromLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventCatchHopper;
import org.usfirst.frc.team1736.robot.auto.AutoEventCrossBaseLine;
import org.usfirst.frc.team1736.robot.auto.AutoEventDriveToCenterLift;
import org.usfirst.frc.team1736.robot.auto.AutoEventGetToDaHoppaLeft;
import org.usfirst.frc.team1736.robot.auto.AutoEventGetToDaHoppaRight;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveLeftOffWall;
import org.usfirst.frc.team1736.robot.auto.AutoEventMoveRightOffWall;
import org.usfirst.frc.team1736.robot.auto.AutoEventOpenGearMechanism;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootNoVision;
import org.usfirst.frc.team1736.robot.auto.AutoEventShootWithVision;
import org.usfirst.frc.team1736.robot.auto.AutoEventTest;

public class Autonomous {
	Calibration autoMode;

	String autoModeName = "Not Initalized";

	int mode;

	public Autonomous() {
		autoMode = new Calibration("Auto Mode", 0, 0, 20);
		mode = 100; // A number I think we will never use
	}

	public void updateAutoSelection() {
		if((int) Math.round(autoMode.get()) != mode) {
			mode = (int) Math.round(autoMode.get());

			// The following must be aligned to the below selection
			switch(mode) {
			case 1: // drive forward across base line
				autoModeName = "Cross Baseline";
				break;
			case 2: // put a gear on the center lift
				autoModeName = "Gear No X";
				break;
			case 3: // put a gear on the center lift
				autoModeName = "Gear X Left";
				break;
			case 4: // put a gear on the center lift
				autoModeName = "Gear X Right";
				break;
			case 5: // Shoot without vision alignment or motion
				autoModeName = "No Move Shoot";
				break;
			case 6: // Move off the blue wall, vision align and shoot
				autoModeName = "Move Right Vision Shoot";
				break;
			case 7: // move off the red wall, vision align and shoot
				autoModeName = "Move Left Vision Shoot";
				break;
			case 8: // GET TO DA HOPPA (and shoot) (robot placed backwards, moves right from drivers perspective)
				autoModeName = "TO DA HOPPA Right";
				break;
			case 9: // GET TO DA HOPPA (and shoot) (robot placed backwards, moves left from drivers perspective)
				autoModeName = "TO DA HOPPA Left";
				break;
			case 10: // Shoot without vision alignment or motion
				autoModeName = "Test - DO NOT USE!";
				break;
			default: // Do nothing
				autoModeName = "Do Nothing";
				break;
			}
			System.out.println("[Auto] New mode selected: " + autoModeName);
		}
	}

	/**
	 * Main setup method for autonomous. Should be called before actually updating autonomous.
	 * Determines which mode to use, sets up the auto events in the timeline, runs path generation,
	 * Resets integrators, and starts the scheduler.
	 */
	public void executeAutonomus() {
		System.out.println("[Auto] Initalizing " + autoModeName + " auton routine.");

		AutoSequencer.clearAllEvents();

		DriveTrain.getInstance().getFrontLeftCTRL().resetIntegrators();
		DriveTrain.getInstance().getFrontRightCTRL().resetIntegrators();
		DriveTrain.getInstance().getRearLeftCTRL().resetIntegrators();
		DriveTrain.getInstance().getRearRightCTRL().resetIntegrators();

		switch(mode) {
		case 1: // drive forward across base line
			AutoEventCrossBaseLine driveForward = new AutoEventCrossBaseLine();
			AutoSequencer.addEvent(driveForward);
			break;
		case 2: // put a gear on the center lift, straight Back
			AutoEventDriveToCenterLift gearDeliverC = new AutoEventDriveToCenterLift();
			AutoSequencer.addEvent(gearDeliverC);
			AutoEventOpenGearMechanism openGearC = new AutoEventOpenGearMechanism();
			AutoSequencer.addEvent(openGearC);
			AutoEventBackAwayFromLiftNoCross backAwayC = new AutoEventBackAwayFromLiftNoCross();
			AutoSequencer.addEvent(backAwayC);
			break;
		case 3: // put a gear on the center lift, go to the left
			AutoEventDriveToCenterLift gearDeliverL = new AutoEventDriveToCenterLift();
			AutoSequencer.addEvent(gearDeliverL);
			AutoEventOpenGearMechanism openGearL = new AutoEventOpenGearMechanism();
			AutoSequencer.addEvent(openGearL);
			AutoEventBackAwayLeftFromLift backAwayL = new AutoEventBackAwayLeftFromLift();
			AutoSequencer.addEvent(backAwayL);
			break;
		case 4: // put a gear on the center lift, go to the right
			AutoEventDriveToCenterLift gearDeliverR = new AutoEventDriveToCenterLift();
			AutoSequencer.addEvent(gearDeliverR);
			AutoEventOpenGearMechanism openGearR = new AutoEventOpenGearMechanism();
			AutoSequencer.addEvent(openGearR);
			AutoEventBackAwayRightFromLift backAwayR = new AutoEventBackAwayRightFromLift();
			AutoSequencer.addEvent(backAwayR);
			break;
		case 5: // drive forward across base line
			AutoEventShootNoVision olShoot = new AutoEventShootNoVision();
			AutoSequencer.addEvent(olShoot);
			break;
		case 6:
			AutoEventMoveRightOffWall driveRightOffWall = new AutoEventMoveRightOffWall();
			AutoSequencer.addEvent(driveRightOffWall);
			AutoEventShootWithVision shootNow = new AutoEventShootWithVision();
			AutoSequencer.addEvent(shootNow);
			break;
		case 7:
			AutoEventMoveLeftOffWall driveLeftOffWall = new AutoEventMoveLeftOffWall();
			AutoSequencer.addEvent(driveLeftOffWall);
			AutoEventShootWithVision shootNow2 = new AutoEventShootWithVision();
			AutoSequencer.addEvent(shootNow2);
			break;
		case 8:
			AutoEventGetToDaHoppaRight getToHoppaR = new AutoEventGetToDaHoppaRight();
			AutoEventCatchHopper catchR = new AutoEventCatchHopper();
			AutoEventShootNoVision shootR = new AutoEventShootNoVision();
			AutoSequencer.addEvent(getToHoppaR);
			AutoSequencer.addEvent(catchR);
			AutoSequencer.addEvent(shootR);
			break;
		case 9:
			AutoEventGetToDaHoppaLeft getToHoppaL = new AutoEventGetToDaHoppaLeft();
			AutoEventCatchHopper catchL = new AutoEventCatchHopper();
			AutoEventShootNoVision shootL = new AutoEventShootNoVision();
			AutoSequencer.addEvent(getToHoppaL);
			AutoSequencer.addEvent(catchL);
			AutoSequencer.addEvent(shootL);
			break;
		case 10:
			AutoEventTest swagCross = new AutoEventTest();
			AutoSequencer.addEvent(swagCross);
			break;
		default: // Do nothing
			break;
		}

		AutoSequencer.start();
	}

	/**
	 * Wrapper for the auto sequence update. Should be called during autonomous periodic
	 */
	public void update() {
		AutoSequencer.update();
	}

	/**
	 * Wrapper for the auto routine ender. Should be called during disabled init just in case we stop auto partway through.
	 */
	public void stop() {
		AutoSequencer.stop();
	}
}
