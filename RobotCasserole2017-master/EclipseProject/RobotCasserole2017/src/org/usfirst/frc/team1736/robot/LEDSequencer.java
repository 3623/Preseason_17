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

import java.awt.Component;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;

import org.usfirst.frc.team1736.lib.LEDs.CasseroleLEDInterface;
import org.usfirst.frc.team1736.lib.LEDs.DesktopTestLEDs;
import org.usfirst.frc.team1736.lib.LEDs.DotStarsLEDStrip;

public class LEDSequencer {
	private static LEDSequencer seqLocal = null;

	private volatile LEDSwitchCase cur_pattern;

	private static final boolean desktop_sim = false;

	public enum LEDSwitchCase {
		OFF, SMOOTH_SWEEP, SMOOTH_RAINBOW, SMOOTH_RED_WHITE, SPARKLE_WHITE, SPARKLE_RED_WHITE, SPARKLE_RAIN, CYLON, COMET_RED, COMET_RAIN, BOUNCE, GEAR, FUEL, CAPN, TEST, SMOOTH_GREEN, SMOOTH_BLUE, BLUE_GREEN_SWEEP
	}

	static CasseroleLEDInterface ledstrip; // interface so that we can swap between desktop and robot
	Timer timerThread;

	double loop_counter;

	public static LEDSequencer getInstance() {
		if(seqLocal == null) {
			seqLocal = new LEDSequencer();
		}
		return seqLocal;
	}

	private LEDSequencer() {
		if(desktop_sim) {
			ledstrip = new DesktopTestLEDs(RobotConstants.NUM_LEDS_TOTAL);
		}
		else {
			ledstrip = new DotStarsLEDStrip(RobotConstants.NUM_LEDS_TOTAL);
		}

		cur_pattern = LEDSwitchCase.OFF;

		loop_counter = 0;

		// Start LED animation thread in background.
		timerThread = new java.util.Timer("LED Sequencer Update");
		timerThread.schedule(new LEDBackgroundUpdateTask(this), (long) (CasseroleLEDInterface.m_update_period_ms), (long) (CasseroleLEDInterface.m_update_period_ms));
	}

	public void update() {
		switch(cur_pattern) {
		case OFF:
			allOff();
			break;

		case TEST:
			testPattern();
			break;

		case SMOOTH_SWEEP:
			smoothStripSweep();
			break;

		case SMOOTH_RAINBOW:
			smoothRainbowCycle();
			break;

		case SMOOTH_RED_WHITE:
			smoothRedWhiteCycle();
			break;

		case SPARKLE_WHITE:
			sparkleWhite();
			break;

		case SPARKLE_RED_WHITE:
			sparkleRedWhite();
			break;

		case SPARKLE_RAIN:
			sparkleRainbow();
			break;

		case CYLON:
			cylon();
			break;

		case COMET_RED:
			cometRed();
			break;

		case COMET_RAIN:
			cometRainbow();
			break;

		case BOUNCE:
			bounce();
			break;

		case GEAR:
			gearSignal();
			break;

		case FUEL:
			fuelSignal();
			break;

		case CAPN:
			capnjack();
			break;

		case SMOOTH_GREEN:
			smoothGreenCycle();
			break;

		case BLUE_GREEN_SWEEP:
			smoothBlueGreenSweep();
			break;

		case SMOOTH_BLUE:
			smoothBlueCycle();
			break;
		}

		// smoothStripSweep();
		// smoothRainbowCycle();
		// smoothRedWhiteCycle();
		// sparkleWhite();
		// sparkleRedWhite();
		// sparkleRainbow();
		// cylon();
		// cometRed();
		// cometRainbow();
		// bounce();
		// gearSignal();
		// fuelSignal();
		// capnjack();
		// smoothGreenCycle();
		// smoothBlueGreenSweep();
		// smoothBlueCycle();

		loop_counter++;
	}

	// Shut them all down!!!
	@SuppressWarnings("unused")
	private void allOff() {
		ledstrip.clearColorBuffer();
	}

	// Shut them all down!!!
	@SuppressWarnings("unused")
	private void testPattern() {
		ledstrip.clearColorBuffer();
		ledstrip.setLEDColor(((int) loop_counter + 0) % RobotConstants.NUM_LEDS_TOTAL, 1, 0, 0);
		ledstrip.setLEDColor(((int) loop_counter + 1) % RobotConstants.NUM_LEDS_TOTAL, 0, 1, 0);
		ledstrip.setLEDColor(((int) loop_counter + 2) % RobotConstants.NUM_LEDS_TOTAL, 0, 0, 1);
	}

	// shift through all colors
	@SuppressWarnings("unused")
	private void smoothRainbowCycle() {
		final double period = 200; // Bigger makes it change color slower

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double hue = Math.abs((loop_counter) % period - period / 2) / (period / 2);
			ledstrip.setLEDColorHSL(led_idx, hue, 1, 0.5);
		}
	}

	// shift through Casserole red/white
	@SuppressWarnings("unused")
	private void smoothRedWhiteCycle() {
		final double period = 200; // Bigger makes it change color slower

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double lightness = Math.abs((loop_counter) % period - period / 2) / (period / 2);
			ledstrip.setLEDColorHSL(led_idx, 0, 1, lightness);
		}
	}

	// Similar to 2016 - does a smooth sweep of casserole red/white stripes
	@SuppressWarnings("unused")
	private void smoothStripSweep() {
		final double width = 2.0; // bigger means wider color strips
		final double period = 5.0; // bigger means slower cycle
		final double edgeSharpness = 1.0; // bigger means less blurred edges between stripe colors

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double not_red_comp = Math.min(1, Math.max(0, (0.5 + edgeSharpness * Math.sin((led_idx / width + loop_counter / period)))));
			ledstrip.setLEDColor(led_idx, 1, not_red_comp, not_red_comp);
		}
	}

	// shiny blips of white
	@SuppressWarnings("unused")
	private void sparkleWhite() {
		final double density = 0.05; // 0 means never on, 1 means always on

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double rand = Math.random();
			if(rand < density) {
				ledstrip.setLEDColor(led_idx, 1, 1, 1);
				led_idx++;// make sure we separate blips by at least one space
			}
			else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}

	// shiny blips of white and red
	@SuppressWarnings("unused")
	private void sparkleRedWhite() {
		final double density = 0.1; // 0 means never on, 1 means always on

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double rand = Math.random();
			if(rand < density) {
				if(rand < density / 2) {
					ledstrip.setLEDColor(led_idx, 1, 1, 1);
				}
				else {
					ledstrip.setLEDColor(led_idx, 1, 0, 0);
				}
				led_idx++;// make sure we separate blips by at least one space
			}
			else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}

	// shiny blips of all colors
	@SuppressWarnings("unused")
	private void sparkleRainbow() {
		final double density = 0.2; // 0 means never on, 1 means always on

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double rand = Math.random();
			if(rand < density) {
				ledstrip.setLEDColorHSL(led_idx, rand / density, 1, 0.5);
				led_idx++;// make sure we separate blips by at least one space
			}
			else {
				ledstrip.setLEDColor(led_idx, 0, 0, 0);
			}
		}
	}

	// Evil robots of battelstar galactica fame
	@SuppressWarnings("unused")
	private void cylon() {
		final double width = 2.0; // bigger means wider on-width
		final int period = 50; // bigger means slower cycle

		double midpoint = (double) (Math.abs(((loop_counter) % period) - period / 2)) / ((double) (period / 2.0)) * (RobotConstants.NUM_LEDS_TOTAL);

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double red_val = Math.max(0, Math.min(1.0, 1 - (0.15 * Math.pow((midpoint - led_idx), 2))));
			ledstrip.setLEDColor(led_idx, red_val, 0, 0);
		}
	}

	// red comet shoots across the sky
	@SuppressWarnings("unused")
	private void cometRed() {
		final double width = 12.0; // bigger means wider on-width
		final int period = 30; // bigger means slower cycle

		double red_val;

		double endpoint = (double) (((loop_counter) % period) / ((double) period)) * ((RobotConstants.NUM_LEDS_TOTAL + width * 10) / 2.0);

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			if(led_idx <= endpoint + 2) {
				red_val = Math.max(0, Math.min(1.0, (1 - (endpoint - led_idx) / width)));
			}
			else {
				red_val = 0;
			}
			ledstrip.setLEDColor(led_idx, red_val, 0, 0);
		}
	}

	// Colorful comets on a dark red background
	@SuppressWarnings("unused")
	private void cometRainbow() {
		final double width = 5.0; // bigger means wider on-width
		final int period = 30; // bigger means slower cycle

		double val;

		double endpoint = (double) (((loop_counter) % period) / ((double) period)) * ((RobotConstants.NUM_LEDS_TOTAL + width * 10));

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			if(led_idx <= endpoint + 2) {
				val = Math.max(0, Math.min(1.0, (1 - (endpoint - led_idx) / width)));
			}
			else {
				val = 0;
			}
			ledstrip.setLEDColorHSL(led_idx, val, 1, (val * 0.5) + 0.25);
		}
	}

	private void smoothGreenCycle() {
		final double period = 54; // Bigger makes it change color slower

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double greeness = Math.abs((loop_counter) % period - period / 2) / (period / 2);
			ledstrip.setLEDColor(led_idx, 0, greeness, 0);
		}
	}

	private void smoothBlueCycle() {
		final double period = 54; // Bigger makes it change color slower

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double blueness = Math.abs((loop_counter) % period - period / 2) / (period / 2);
			ledstrip.setLEDColor(led_idx, 0, 0, blueness);
		}
	}

	// Globals needed for bouncing
	double pos1 = 5;
	double pos2 = 8;
	double vel1 = 0.4;
	double vel2 = -0.8;

	// Bouncing Balls
	@SuppressWarnings("unused")
	private void bounce() {
		// Super simple mostly-elastic colisison model
		if(pos1 <= 0 | pos1 >= RobotConstants.NUM_LEDS_TOTAL) {
			vel1 = -vel1;
			pos1 = Math.min(RobotConstants.NUM_LEDS_TOTAL, Math.max(0, pos1));
		}
		if(pos2 <= 0 | pos2 >= RobotConstants.NUM_LEDS_TOTAL) {
			vel2 = -vel2;
			pos2 = Math.min(RobotConstants.NUM_LEDS_TOTAL, Math.max(0, pos2));
		}
		if(Math.abs(pos1 - pos2) < Math.max(Math.abs(vel1), Math.abs(vel2))) { // collision
			double tmp = vel1 * 0.95;
			vel1 = vel2 * 0.95;
			vel2 = tmp;
		}
		if(Math.abs(vel1) < 0.01 & Math.abs(vel2) < 0.01) { // reset
			vel1 = Math.random();
			vel2 = -Math.random();
			pos1 = 5;
			pos2 = 15;
		}

		if(pos1 > 0) {
			// vel1 = vel1 - 0.05;
		}
		if(pos2 > 0) {
			// vel2 = vel2 - 0.05;
		}

		pos1 += vel1;
		pos2 += vel2;

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double val1 = Math.max(0, Math.min(1.0, 1 - (0.75 * Math.pow((pos1 - led_idx), 2))));
			double val2 = Math.max(0, Math.min(1.0, 1 - (0.75 * Math.pow((pos2 - led_idx), 2))));
			ledstrip.setLEDColor(led_idx, val1, 0, val2);
		}
	}

	private void gearSignal() {
		for(int led_idx = 0; led_idx < (RobotConstants.NUM_LEDS_TOTAL); led_idx++) {
			if((led_idx % 2 * 5) < 5) {
				ledstrip.setLEDColor(led_idx, .76, .91, .28);
				ledstrip.setLEDColor(RobotConstants.NUM_LEDS_TOTAL - (led_idx) - 1, .76, .91, .28);
			}
			else {
				ledstrip.setLEDColor(led_idx, .91, .2, .84);
				ledstrip.setLEDColor(RobotConstants.NUM_LEDS_TOTAL - (led_idx) - 1, .91, .2, .84);
			}
		}
	}

	private void fuelSignal() {
		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			if((led_idx % (2 * 7)) < 7) {
				ledstrip.setLEDColor(led_idx, .89, .71, .06);
				ledstrip.setLEDColor(RobotConstants.NUM_LEDS_TOTAL - (led_idx) - 1, .89, .71, .06);
			}
			else {
				ledstrip.setLEDColor(led_idx, 1, 0, 0);
				ledstrip.setLEDColor(RobotConstants.NUM_LEDS_TOTAL - (led_idx) - 1, 1, 0, 0);
			}
		}
	}

	private void capnjack() {
		final int period = 50; // bigger means slower cycle

		double midpoint = (double) (Math.abs(((loop_counter) % period) - period / 2)) / ((double) (period / 2.0)) * (RobotConstants.NUM_LEDS_TOTAL);

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			if((led_idx % (2 * 1)) < 1) {
				double red_val = Math.max(0, Math.min(1.0, 1 - (0.15 * Math.pow((midpoint - led_idx), 2))));
				ledstrip.setLEDColor(led_idx, red_val, 0, 0);
			}
			else {
				ledstrip.setLEDColor(led_idx, 0, 0.7, 0);
			}
		}
	}

	private void smoothBlueGreenSweep() {
		final double width = 1.0; // bigger means wider color strips
		final double period = 4.0; // bigger means slower cycle
		final double edgeSharpness = 1.0; // bigger means less blurred edges between stripe colors

		for(int led_idx = 0; led_idx < RobotConstants.NUM_LEDS_TOTAL; led_idx++) {
			double not_green_comp = Math.min(1, Math.max(0, (0.3 + edgeSharpness * Math.sin((led_idx / width + loop_counter / period)))));
			ledstrip.setLEDColor(led_idx, 0.1, 1, not_green_comp);
		}
	}

	public void setGearDesiredPattern() {
		cur_pattern = LEDSwitchCase.SMOOTH_GREEN;
		return;
	}

	public void setFuelDesiredPattern() {
		cur_pattern = LEDSwitchCase.SMOOTH_BLUE;
		return;
	}

	public void setBothDesiredPattern() {
		cur_pattern = LEDSwitchCase.BLUE_GREEN_SWEEP;
		return;
	}

	public void setNoneDesiredPattern() {
		cur_pattern = LEDSwitchCase.COMET_RAIN;
	}

	public void setAutonPattern() {
		cur_pattern = LEDSwitchCase.SPARKLE_RED_WHITE;
		return;
	}

	public void setDisabledPattern() {
		cur_pattern = LEDSwitchCase.SMOOTH_SWEEP;
		return;
	}

	public void pickRandomPattern() {
		// cur_pattern = LEDSwitchCase.values()[(int)(Math.random()*((double)(LEDSwitchCase.values().length)))];
		cur_pattern = LEDSwitchCase.SMOOTH_SWEEP;
	}

	// Java multithreading magic. Do not touch.
	// Touching will incour the wrath of Cthulhu, god of java and LED Strips.
	// May the oceans of 1's and 0's rise to praise him.
	private class LEDBackgroundUpdateTask extends TimerTask {
		private LEDSequencer m_sequencer;

		public LEDBackgroundUpdateTask(LEDSequencer sequencer) {
			if(sequencer == null) {
				throw new NullPointerException("Given Desktop LEDs Controller Class was null");
			}

			m_sequencer = sequencer;
		}

		@Override
		public void run() {
			m_sequencer.update();
		}
	}

	// Main method for testing locally on a PC
	@SuppressWarnings("unused")
	public static void main(String[] args) {
		LEDSequencer seq = new LEDSequencer();

		seq.cur_pattern = LEDSwitchCase.TEST;

		JFrame frame = new JFrame("LED Test");
		frame.setSize(850, 200);
		frame.setVisible(true);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		if(desktop_sim) {
			frame.add((Component) ledstrip); // uncomment this to do a desktop test
		}
	}
}
