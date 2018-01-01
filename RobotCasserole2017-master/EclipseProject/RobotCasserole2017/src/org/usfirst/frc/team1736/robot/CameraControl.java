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

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.HashMap;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;

public class CameraControl {
	private HashMap<String, UsbCamera> driverCameras = new HashMap<String, UsbCamera>();
	private MjpegServer driverStream = null;

	/**
	 * Class to control the webcam server run on the roboRIO. Configures the webcam size & framerate to
	 * limit bandwidth. Keeps the exposure settings right for the vision process camera.
	 */
	public CameraControl() {
		UsbCamera visionProcCam;
		UsbCamera logitech1;
		UsbCamera logitech2;

		// Start Vision Process Camera and two driver cameras
		visionProcCam = new UsbCamera("Lifecam", getCameraDeviceNumber(RobotConstants.LIFECAM_USB_DEVICE_ID_SERIAL)); // vision process
		logitech1 = new UsbCamera("Logitech_1", getCameraDeviceNumber(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_1)); // Forward alignment camera (driver)
		logitech2 = new UsbCamera("Logitech_2", getCameraDeviceNumber(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_2)); // Gear alignment camera (driver)
		driverCameras.put(RobotConstants.LIFECAM_USB_DEVICE_ID_SERIAL, visionProcCam);
		driverCameras.put(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_1, logitech1);
		driverCameras.put(RobotConstants.LOGITECH_USB_DEVICE_ID_SERIAL_2, logitech2);

		// Configure and stream vision processing camera
		visionProcCam.setFPS(15); // this seems to not work for some reason??? meh
		visionProcCam.setResolution(RobotConstants.VISION_X_PIXELS, RobotConstants.VISION_Y_PIXELS);
		visionProcCam.setExposureManual(5); // friendly for computers, nasty for humans
		visionProcCam.setWhiteBalanceManual(9000);
		MjpegServer visionCamServerHighRes = new MjpegServer("VisionProcCamServer", 1181);
		visionCamServerHighRes.setSource(visionProcCam);

		System.out.println(visionProcCam.getDescription());

		// configure and stream Driver Cameras
		logitech1.setResolution(320, 240);
		logitech1.setFPS(15);
		logitech2.setResolution(320, 240);
		logitech2.setFPS(15);
		driverStream = new MjpegServer("DriverCamServer", 1182);
		driverStream.setSource(logitech2);

		// Confirming stuff is plugged in and unicorns and such
		System.out.println(logitech1.getDescription());
		System.out.println(logitech2.getDescription());
	}

	/**
	 * magic.
	 * 
	 * @param serialId
	 * @return
	 */
	public static int getCameraDeviceNumber(String serialId) {
		for(int i = 0; i < 4; i++) {
			try {
				Process p = Runtime.getRuntime().exec("udevadm info --query=all --name=/dev/video" + i + " | grep ID_SERIAL=");
				p.waitFor();
				BufferedReader reader = new BufferedReader(new InputStreamReader(p.getInputStream()));
				String line = "";
				while((line = reader.readLine()) != null) {
					if(line.contains(serialId))
						return i;
					if(line.contains("device node not found"))
						return -1;
				}
			}
			catch(Exception e) {
				e.printStackTrace();
				return -1;
			}
		}
		return -1;
	}

	/**
	 * Uses magic to keep the cameras from swapping. Take that evil usb camera genie.
	 * 
	 * @param serialId
	 */
	public void setDriverCamera(String serialId) {
		if(driverStream.getSource().equals(driverCameras.get(serialId)))
			return;
		driverStream.setSource(driverCameras.get(serialId));
	}
}
