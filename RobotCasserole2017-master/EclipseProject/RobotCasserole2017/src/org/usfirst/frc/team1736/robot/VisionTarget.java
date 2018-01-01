package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Util.MapLookup2D;

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

/**
 * Class to convert a vision target pixel location to a distance/angle.
 *
 */
public class VisionTarget {

	private boolean targetFound = false;
	private double bestX = 0;
	private double bestY = 0;
	private double bestWidth = 0;
	private boolean distanceValid = false;

	private final double TANGENT_CAMERA_FOV_X = Math.tan(Math.toRadians(RobotConstants.CAMERA_FOV_X_DEG / 2.0));
	private MapLookup2D yPxToDist;

	public VisionTarget() {
		yPxToDist = new MapLookup2D();
		// Populate LUT for vision distance with whatever we find
		// First number is pixel ratio, second number is feed
		yPxToDist.insertNewPoint(0.0 / 720.0, 3.0);
	}

	/**
	 * Given the width/x/y of the best candidate for top target, update relevant physical parameters
	 * 
	 * @param foundTgt
	 *            true if a target was seen, false if n0t
	 * @param bestX
	 *            x pixel location of top centroid
	 * @param bestY
	 *            y pixel location of top centroid
	 * @param bestWidth
	 *            width of top centroid
	 */
	public void updateTarget(boolean targetFound, double bestX, double bestY, double bestWidth)	{
		if(targetFound)
		{
			this.targetFound = true;
			this.bestX = bestX;
			this.bestY = bestY;
			this.bestWidth = bestWidth;
		}
		else
		{
			this.targetFound = false;
		}
	}

	public boolean isTargetFound() {
		return targetFound;
	}

	public double getTopTargetXPixelPos() {
		return bestX;
	}

	public double getTopTargetYPixelPos() {
		return bestY;
	}

	public double getEstTargetDistanceFt() {
		double cam_to_tgt_dist_ft = (RobotConstants.TGT_WIDTH_FT * RobotConstants.VISION_X_PIXELS) / (2.0 * bestWidth * TANGENT_CAMERA_FOV_X); // From https://wpilib.screenstepslive.com/s/4485/m/24194/l/288985-identifying-and-processing-the-targets
		double cam_to_tgt_dist_ft_sqrd = Math.pow(cam_to_tgt_dist_ft, 2);
		final double visionTgtHeightSqrd = Math.pow(RobotConstants.HIGH_GOAL_VISION_TARGET_HEIGHT_FT, 2);

		// We need to calculate distance along the ground, so use pythagorean theorem to calculate floor distance, given target height.
		// ensure the square root will have a positive result (otherwise something wacky is going on)
		if(cam_to_tgt_dist_ft_sqrd > visionTgtHeightSqrd) {
			distanceValid = true;
			return Math.sqrt(cam_to_tgt_dist_ft_sqrd - visionTgtHeightSqrd);
		} else {
			distanceValid = false;
			return -1;
		}
	}

	public double getLookupTargetDistanceFt() {
		return yPxToDist.lookupVal(((double) bestY) / ((double) RobotConstants.VISION_Y_PIXELS));
	}

	public double getTargetOffsetDegrees() {
		return (bestX - RobotConstants.VISION_X_PIXELS / 2) * (RobotConstants.CAMERA_FOV_X_DEG / RobotConstants.VISION_X_PIXELS);
	}

	public boolean isDistanceValid() {
		return distanceValid;
	}

}
