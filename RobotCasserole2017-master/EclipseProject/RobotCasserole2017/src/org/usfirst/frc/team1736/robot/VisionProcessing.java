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

import java.util.ArrayList;

import org.usfirst.frc.team1736.lib.CoProcessor.VisionListener;

import edu.wpi.first.wpilibj.Timer;

public class VisionProcessing {
	private static VisionProcessing visionProcessing = null;
	private VisionListener listener;

	ArrayList<Integer> topTargets = new ArrayList<Integer>(15);
	ArrayList<Integer> bottomTargets = new ArrayList<Integer>(15);

	private double prevFrameCount;

	// local Constants
	private final double CURVATURE_FUDGE_FACTOR = 1.75; // Accounts for the fact the camera angle plus cylinder shape makes for a curved (not rectangular) target. I feel like this is dubious math, but it seems to help for now....

	private final double Exp_AspectRatio_Top = 15.0 / (4.0 * CURVATURE_FUDGE_FACTOR);
	private final double Exp_AspectRatio_Bottom = 15.0 / (2.0 * CURVATURE_FUDGE_FACTOR);
	private final double Exp_InfillRatio_Top = 0.75; // An educated guess
	private final double Exp_InfillRatio_Bottom = 0.75; // An educated guess

	private VisionTarget target = null;

	private double curHeuristic = 0;

	public static synchronized VisionProcessing getInstance() {
		if(visionProcessing == null)
			visionProcessing = new VisionProcessing();
		return visionProcessing;
	}

	/**
	 * Class to interpret the blobs of green returned from the vision target identification subsystem
	 * (currently Beaglebone + webcam)
	 */
	private VisionProcessing() {
		prevFrameCount = 0;

		listener = new VisionListener(RobotConstants.COPPROCESSOR_LISTEN_ADDRESS, RobotConstants.COPROCESSOR_LISTEN_PORT);
		listener.start();
		target = new VisionTarget();
	}

	/**
	 * Should be called during the periodic update method to evaluate the new info from the coprocessor.
	 * Grabs the latest info from the coprocessor setup, and calls the heuristic update algorithm if
	 * we found a new frame
	 */
	public void update() {
		// Sample latest available data
		listener.sampleLatestData();

		if(prevFrameCount != listener.getFrameCounter()) {
			// If we have a new frame since the last time we processed, run the processing algorithm.
			alg1();
		}
	}

	/**
	 * Simple heuristic algorithm, assumes both a top and bottom can be located.
	 */
	@SuppressWarnings("unused")
	private void alg0() {
		topTargets.clear();
		bottomTargets.clear();

		double AspectRatio;
		double Pct_Error_Top;
		double Pct_Error_Bottom;
		double Heuristic;
		int Best_Top = -1;
		int Best_Bottom = -1;
		double Best_Heuristic = 9001;

		for(int i = 0; i < listener.getNumTargetsObserved(); i++) {
			AspectRatio = listener.getWidth(i) / listener.getHeight(i);
			Pct_Error_Top = Math.abs((AspectRatio - Exp_AspectRatio_Top) / Exp_AspectRatio_Top);
			Pct_Error_Bottom = Math.abs((AspectRatio - Exp_AspectRatio_Bottom) / Exp_AspectRatio_Bottom);

			if(Pct_Error_Top > Pct_Error_Bottom) {
				bottomTargets.add(i);
			}
			else {
				topTargets.add(i);
			}
		}

		for(int i = 0; i < bottomTargets.size(); i++) {
			for(int j = 0; j < topTargets.size(); j++) {

				Heuristic = Math.abs(listener.getY(bottomTargets.get(i)) - listener.getY(topTargets.get(j)));

				if(Heuristic < Best_Heuristic) {
					Best_Heuristic = Heuristic;
					Best_Top = bottomTargets.get(i);
					Best_Bottom = topTargets.get(j);
				}
			}
		}

		if(Best_Top != -1 & Best_Bottom != -1) {
			target.updateTarget(true, listener.getX(Best_Top), listener.getY(Best_Top), listener.getWidth(Best_Top));
		} else {
			target.updateTarget(false, 0, 0, 0);
		}
	}

	/**
	 * More-advanced heuristic calculation, which does not try to sort by upper/lower target first
	 * Note this is O(n^2) complexity (actually n*n-1 calculation cycles for n targets), so use caution if many targets are visible.
	 * 
	 * This algorithm evaluates many different relationships between the two candidate target blobs, and returns the best pair.
	 */
	private void alg1() {
		double Heuristic = -1;
		int Best_Top = -1;
		int Best_Bottom = -1;
		double Best_Heuristic = Double.MAX_VALUE;

		double x_pos_error;
		double y_pos_error;
		double y_sep_exp;
		double top_ar_error;
		double bottom_ar_error;
		double width_error;
		double height_error;
		double top_infill_error;
		double bottom_infill_error;
		double i_like_big_targets_and_i_cannot_lie;

		for(int i = 0; i < listener.getNumTargetsObserved(); i++) { // i is top target iter
			for(int j = 0; j < listener.getNumTargetsObserved(); j++) { // j is bottom target iter

				if(i == j) {
					// Cannot compare a target to itself
					continue;
				}
				x_pos_error = Math.abs(listener.getX(i) - listener.getX(j)); // expect X positions to be aligned
				y_sep_exp = (double) Math.round((listener.getHeight(i) * 3.0 / 2.0 + listener.getHeight(j) / 2) / CURVATURE_FUDGE_FACTOR);
				y_pos_error = Math.abs((listener.getY(j) - listener.getY(i)) - y_sep_exp); // Expect Top to be above Bottom (top's y < bottom's y) by an assumed distance
				width_error = Math.abs(listener.getWidth(i) - listener.getWidth(j)); // Expect same width
				height_error = Math.abs(listener.getHeight(i) - listener.getHeight(j) * 2.0); // expect top height to be double bottom height
				top_ar_error = Math.abs((listener.getWidth(i) / listener.getHeight(i)) - Exp_AspectRatio_Top); // Expect certain aspect ratios
				bottom_ar_error = Math.abs((listener.getWidth(j) / listener.getHeight(j)) - Exp_AspectRatio_Bottom);
				top_infill_error = Math.abs((listener.getArea(i) / (listener.getWidth(i) * listener.getHeight(i))) - Exp_InfillRatio_Top);
				bottom_infill_error = Math.abs((listener.getArea(j) / (listener.getWidth(j) * listener.getHeight(j))) - Exp_InfillRatio_Bottom);
				i_like_big_targets_and_i_cannot_lie = 100000 * 1 / (listener.getArea(i) + listener.getArea(j));

				// The better the target is, the smaller Heuristic should be
				Heuristic = x_pos_error * 10.0 + // We want the top/bottom centroids to be aligned in the X direction
						y_pos_error * 10.0 + // Given the heights of the top/bottom, we expect a certain offset in the centroids in the Y direction
						width_error * 10.0 + // We expect the top/bottom to have the same width
						height_error * 5.0 + // We expect the top to have twice the height of the bottom
						top_ar_error * 1.0 + // We expect the top to have a certain aspect ratio
						bottom_ar_error * 1.0 + // We expect the bottom to have a certain aspect ratio
						top_infill_error * 1.0 + // We expect the top to have a certain infill percentage
						bottom_infill_error * 1.0 + // We expect the bottom to have a certain infill percentage
						i_like_big_targets_and_i_cannot_lie; // Bigger targets are better.

				// We expect only one target possible. Pick the best.
				if(Heuristic < Best_Heuristic) {
					Best_Heuristic = Heuristic;
					Best_Top = i;
					Best_Bottom = j;
				}
			}
		}

		if(Best_Top != -1 & Best_Bottom != -1) {
			target.updateTarget(true, listener.getX(Best_Top), listener.getY(Best_Top), listener.getWidth(Best_Top));
			curHeuristic = Heuristic;
		} else {
			target.updateTarget(false, 0, 0, 0);
		}
	}

	/**
	 * Getter and setter things
	 * 
	 * @return all the things
	 */

	public VisionTarget getTarget()	{
		return target;
	}

	public boolean isOnline() {
		return listener.isCoProcessorAlive();
	}

	public double getCoProcessorFPS() {
		return listener.getFPS();
	}

	public double getCoProcessorCPULoadPct() {
		return listener.getCpuLoad();
	}

	public double getCoProcessorMemLoadPct() {
		return listener.getMemLoad();
	}

	public double getEstCaptureTime() {
		// return listener.getPacketRxSystemTime() - listener.getProcTimeMs()/1000.0 - RobotConstants.EXPECTED_NETWORK_LATENCY_SEC;
		return Timer.getFPGATimestamp() - RobotConstants.TOTAL_VISION_DELAY_S;
	}

	public double getFrameCount() {
		return listener.getFrameCounter();
	}

	public double getVisionProcessTimeMs() {
		return listener.getProcTimeMs();
	}

	public double getNumberOfTargetsObserved() {
		return listener.getNumTargetsObserved();
	}

	public double getCurHeuristic() {
		return curHeuristic;
	}

}
