package org.usfirst.frc.team3623.robot;

import com.ctre.CANTalon;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Do Nothing";
	final String customAuto0 = "Mobility Fancy";
	final String customAuto1 = "Gear: Center Lift";
	final String customAuto2 = "Gear: Left Lift";
	final String customAuto3 = "Gear: Right Lift";
	final String customAuto4 = "Mobility Short";
	
	final String speedModeHigh = "High Speed";
	final String speedModeMedium = "Medium Speed";
	
	final String driveModeField = "Field Oriented";
	final String driveModeRobot = "Robot Oriented";
	
	SendableChooser<String> autoChooser = new SendableChooser<>();
	String autoMode;
	
	SendableChooser<String> speedChooser = new SendableChooser<>();
	public String speedModeSelected;
	public String speedMode;
	
	SendableChooser<String> driveChooser = new SendableChooser<>();
	public String driveMode;
    public String driveModeSelected;
    
	public Joystick driveDirection, driveRotation, operator;
	public Spark LF, RF, LB, RB;
	public CANTalon Lift;
	public Relay LED;
	public RobotDrive driveBase;
	
	public DriverStation DS;
	
	public ADXRS450_Gyro spin;
    public double rotationDif = 0;
    public double driveX, driveY, XDif;
    
    DigitalInput gearTrigger;
  
    BuiltInAccelerometer accel;
    double XValField = 0.0;
    public int accelCounter = 0;
    public double[] accelValues = new double[] {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    
	public static final int FRONT_LEFT_MOTOR = 0;
    public static final int FRONT_RIGHT_MOTOR = 1;
    public static final int BACK_LEFT_MOTOR = 2;
    public static final int BACK_RIGHT_MOTOR = 3;
    public static final double MEDIUM_SPEED = 0.6;
    public static final double PRECISION_SPEED = 0.3;
    public static final int LIFT = 0;
    public static final int LEDS_ON = 1;//turn on the LEDs

    public Timer autoTimer, autoTimer2, backOffTimer;
    
    NetworkTable table;
    public double[] centerX;
    public double contours;
    
    public boolean targetMissed = false;
	public boolean targetFound = false;
	public boolean gearLifted = false;
	public boolean backOff = false;
	public boolean noReturn = false;
	public boolean autoStart = false;
	public boolean auto2Start = false;
	public boolean visionStart = false;
	public boolean timerReset = false;
	public boolean endAuto = true;
	public int stage = 0;
	
	
    
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	
	// Declaring SmartDashboard Objects
	autoChooser.addDefault("Do Nothing", defaultAuto);
	autoChooser.addObject("Mobility Long", customAuto0);
	autoChooser.addObject("Mobility Short", customAuto4);
	autoChooser.addObject("Gear: Center Lift", customAuto1);
	autoChooser.addObject("Gear: Left Lift", customAuto2);
	autoChooser.addObject("Gear: Right Lift", customAuto3);
	SmartDashboard.putData("Auto choices", autoChooser);
	
	speedChooser.addDefault("High Speed", speedModeHigh);
	speedChooser.addObject("Medium Speed", speedModeMedium);
	SmartDashboard.putData("Speed Mode", speedChooser);
	
	driveChooser.addDefault("Field  Oriented", driveModeField);
	driveChooser.addObject("Robot Oriented", driveModeRobot);
	SmartDashboard.putData("Drive Mode", driveChooser);

	CameraServer.getInstance().startAutomaticCapture();
	
	// Declaring drive motors
	LF = new Spark(FRONT_LEFT_MOTOR);
	RF = new Spark(FRONT_RIGHT_MOTOR);
	LB = new Spark(BACK_LEFT_MOTOR);
	RB = new Spark(BACK_RIGHT_MOTOR);
	Lift = new CANTalon(0);
	driveBase = new RobotDrive(LF, LB, RF, RB);

	// Declaring joysticks
	driveDirection = new Joystick(0);
	driveRotation = new Joystick(1);
        operator = new Joystick(2);
        
        // Inverts Left side motors to allow robot to work with Mechanum code
        LF.setInverted(true);
        LB.setInverted(true);
        
        DS = DriverStation.getInstance();
        
 	spin = new ADXRS450_Gyro();       
        // Resets gyro on robot startup
        spin.reset();
        spin.calibrate();
        
        gearTrigger = new DigitalInput(0);
        
        accel = new BuiltInAccelerometer(Accelerometer.Range.k4G);
        
        LED = new Relay(0);
        LED.setDirection(Relay.Direction.kForward);
        LED.set(Relay.Value.kOn);
        
        autoTimer = new Timer();
        autoTimer2 = new Timer();
        backOffTimer = new Timer();
        autoTimer.start();
        autoTimer2.start();
        backOffTimer.start();
        
        table = NetworkTable.getTable("GRIP/myContoursReport");
	}

	/**
	 * This autonomous (along with the autoChooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * autoChooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the autoChooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the autoChooser code above as well.
	 */
	@Override
	public void autonomousInit() {
	autoMode = autoChooser.getSelected();
	// autoMode = SmartDashboard.getString("Auto Selector",
	// defaultAuto);
	System.out.println("Auto selected: " + autoMode);
	LED.set(Relay.Value.kOn);
	autoTimer.reset();
	autoTimer2.reset();
	backOffTimer.reset();
	spin.reset();	
	    
	    targetMissed = false;
	targetFound = false;
	gearLifted = false;
	backOff = false;
	noReturn = false;
	autoStart = false;
	auto2Start = false;
	visionStart = false;
	timerReset = false;
	endAuto = false;
	stage = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
	gearLifted = gearTrigger.get();
	double autoTime = autoTimer.get();
	double autoTime2 = autoTimer2.get();
	double backOffTime = backOffTimer.get();
	SmartDashboard.putNumber("Auto Timer Value", autoTime);
	double matchTime = DS.getMatchTime();
	int matchSecs = ((int)matchTime%60);
	int matchMins = (int)(matchTime/60);
	
	String matchTimeString = matchMins + ":" + matchSecs;
	SmartDashboard.putString("Match Time", matchTimeString);
	SmartDashboard.putString("Drive Mode:", autoMode);
	SmartDashboard.putNumber("Current Heading: ", gyroCorrected());	
	//Accel Debugging Tools
	acceldriveStraight(gyroCorrected(), 1.0);
	SmartDashboard.putNumber("Accel Drive Straight", acceldriveStraight(gyroCorrected(), 1.0));
	SmartDashboard.putNumber("Accel Counter", accelCounter);
	SmartDashboard.putNumber("Accelerometer Value Field", XValField);
	SmartDashboard.putNumber("Accelerometer Value", accel.getX());
	SmartDashboard.putNumber("Contours: ", contours);
	SmartDashboard.putNumber("Stage: ", stage);
	SmartDashboard.putBoolean("targetMissed", targetMissed);
	SmartDashboard.putBoolean("targetFound", targetFound);
	SmartDashboard.putBoolean("gearLifted", gearLifted);
	SmartDashboard.putBoolean("backOff", backOff);
	SmartDashboard.putBoolean("noReturn", noReturn);
	SmartDashboard.putBoolean("autoStart", autoStart);
	SmartDashboard.putBoolean("auto2Start", auto2Start);
	SmartDashboard.putBoolean("visionStart", visionStart);
	SmartDashboard.putNumber("BackoffTimer", backOffTime);
	SmartDashboard.putNumber("XDif Debug", XDif);

	visionpegAlign(1.0);
	if (timerReset == false){//Resets/starts timer once at first time if is executed
	autoTimer.reset();
	timerReset = true;
	}
	
	switch (autoMode) {
	case customAuto0: // Mobility Fancy
	if (autoTime < 1.0){
	driveBase.mecanumDrive_Cartesian(0.0, -0.65*autoTime, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else if (autoTime < 5.0){
	driveBase.mecanumDrive_Cartesian(acceldriveStraight(gyroCorrected(), 1.0), -0.65, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else {
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.4, spin.getAngle());
	}
	break;
	case customAuto1: // Gear Center Lift
	gearLifted = false;
	//Timer continue to reset unless gear is lifted, so time will stay low unless gear is lifted
	//If gear is dropped timer will continue to reset
	if (gearLifted == false){
	backOffTimer.reset();
	}
	// After set time robot will start backing off
	if (backOffTime > 5.0){
	backOff = true;
	}
	else{
	backOff = false;
	}
	
	if (autoTime < 1.0){
	driveBase.mecanumDrive_Cartesian((XDif/700), -0.5*autoTime, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else if (autoTime < 2.9){
	driveBase.mecanumDrive_Cartesian(0.05, -0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else{
	if (backOff == false && gearLifted == false){
	if (contours == 1 && visionStart == false && endAuto == false){//Continues driving at a slower speed until two targets are found
	if(XDif < 0){
	driveBase.mecanumDrive_Cartesian(-0.32, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else if(XDif > 0){
	driveBase.mecanumDrive_Cartesian(0.32, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	stage = 51;
	}
	if (contours == 2 && XDif > 6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.27, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 52;
	}
	else if (contours == 2 && XDif < -6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(-0.27, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 53;
	}
	else if ((contours == 2 && Math.abs(XDif) < 6) || visionStart == true){
	if (autoStart == false){//Resets/starts timer once at first time if is executed
	autoTimer2.reset();
	autoStart = true;
	visionStart = true;//Flag will continue to run this if after it is executed once
	stage = 541;
	}
	else if (autoTime2 < 0.5){
	if (XDif > 3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.09, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 542;
	}
	else if (XDif < -3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(-0.09, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 543;
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 544;
	}
	}
	
	else if (autoTime2 < 1.5){// Drives onto the peg, at this time if targets are found it wont matter, too close to peg
	if(contours == 2){
	driveBase.mecanumDrive_Cartesian((XDif/700), -0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	stage = 545;
	}
	else if (autoTime2 < 2.5 && contours == 2){
	driveBase.mecanumDrive_Cartesian((XDif/800), -0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 546;
	}
	else if (autoTime2 < 7.0){// Waits set time at peg, will back off any time if gear is detected to be picked up
	driveBase.mecanumDrive_Cartesian(0.0, -0.125, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 547;
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 548;
	endAuto = true;
	
	}
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 55;
	endAuto = true;
	}
	else{//Waits to see if it will find a target again
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 56;
	}
	}
	else if (backOff == true){
	if (backOffTime < 6.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.54, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else if (backOffTime < 7.7){
	driveBase.mecanumDrive_Cartesian(-0.7, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else if (autoTime < 11.5){
	driveBase.mecanumDrive_Cartesian(0.0, -0.8, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else {
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	}
	else if (gearLifted == true){
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	}
	break;
	case customAuto2:// Gear Left Lift
	gearLifted = false;
	if (autoTime < 1.0){//Slow start driving, will always run at the start of the match
	driveBase.mecanumDrive_Cartesian(0.0, -0.5*autoTime, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 10;
	}
	else{
	//Logic for deciding if targets were missed or if they were found
	//Will assume targets are missed if they are not found by set time
	//Will start dead reckoning once assumed, but can always switch to
	//Vision based align if targets are found. 
	if (contours == 0 && targetMissed == false && targetFound == false){
	if (autoTime > 4.5){
	targetMissed = true;
	stage = 11;
	}
	else{//Should drive past the peg if no targets are found
	driveBase.mecanumDrive_Cartesian(0.0, -0.38, pointtoRotate(60.0, 0.25), spin.getAngle());
	stage = 12;
	}
	}
	else if (contours != 0 && (gyroCorrected() < 57.0 || gyroCorrected() > 63.0 || autoTime < 2.5)){// Prevents code from thinking target is found at unreallistic times
	driveBase.mecanumDrive_Cartesian(0.0, -0.38, pointtoRotate(60.0, 0.25), spin.getAngle());
	stage = 13;
	}
	else if (contours != 0){
	targetFound = true;
	stage = 14;
	}
	
	//Timer continue to reset unless gear is lifted, so time will stay low unless gear is lifted
	//If gear is dropped timer will continue to reset
	if (gearLifted == false){
	backOffTimer.reset();
	}
	// After set time robot will start backing off
	if (backOffTime > 5.0){
	backOff = true;
	}
	else{
	backOff = false;
	}
	
	//Dead reckoning phase of auto. Back off phase has top priority. Will continue running if no return is triggered
	//despite if a target is found after the point of no return
	if ((targetMissed == true && targetFound == false || noReturn == true) && backOff == false && gearLifted == false){
	stage = 4;
	if (auto2Start == false){//Resets/starts timer once at first time if is executed
	autoTimer2.reset();
	auto2Start = true;
	stage = 41;
	}
	if (autoTime2 < 0.5){// Drives backwards to get in front of peg again
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 42;
	}
	else if (autoTime2 < 1.5){// Drives backwards to get in front of peg again
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 42;
	}
	else if (autoTime2 < 3.0){// Drives onto the peg, at this time if targets are found it wont matter, too close to peg
	noReturn = true;
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 43;
	}
	else if (autoTime2 < 7.0){// Waits set time at peg, will back off any time if gear is detected to be picked up
	driveBase.mecanumDrive_Cartesian(0.0, -0.125, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 44;
	}
	// Will back up no matter what in the last second of auto
	//Multiple thoughts, dead reckoning is assumed to have missed, if so backing up saves that time during tele
	//If dead reckoning hits, will save time to moving that distance in tele
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 45;
	endAuto = true;
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 46;
	}
	}//Will run vision based alignment unless robot starts backing up or dead reckoning is at no return point
	else if (targetFound == true && backOff == false && noReturn == false && gearLifted == false){
	stage = 5;
	if (contours == 1 && visionStart == false && endAuto == false){//Continues driving at a slower speed until two targets are found
	if(XDif < 0){
	driveBase.mecanumDrive_Cartesian(0.0, -0.18, pointtoRotate(60.0, 1.0), spin.getAngle());
	}
	else if(XDif > 0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.18, pointtoRotate(60.0, 1.0), spin.getAngle());
	}
	stage = 51;
	}
	if (contours == 2 && XDif > 6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.0, 0.22, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 52;
	}
	else if (contours == 2 && XDif < -6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.0, -0.22, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 53;
	}
	else if ((contours == 2 && Math.abs(XDif) < 6) || visionStart == true){
	if (autoStart == false){//Resets/starts timer once at first time if is executed
	autoTimer2.reset();
	autoStart = true;
	visionStart = true;//Flag will continue to run this if after it is executed once
	stage = 541;
	}
	else if (autoTime2 < 0.5){
	if (XDif > 3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(-0.09, 0.0, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 542;
	}
	else if (XDif < -3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.09, 0.0, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 543;
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, -0.0, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 544;
	}
	}
	
	else if (autoTime2 < 1.5){// Drives onto the peg, at this time if targets are found it wont matter, too close to peg
	if(contours == 2){
	driveBase.mecanumDrive_Cartesian((XDif/720), -0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	}
	stage = 545;
	}
	else if (autoTime2 < 2.5 && contours == 2){
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 546;
	}
	else if (autoTime2 < 7.0){// Waits set time at peg, will back off any time if gear is detected to be picked up
	driveBase.mecanumDrive_Cartesian(0.0, -0.125, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 547;
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 548;
	endAuto = true;
	}
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 1.0), spin.getAngle()-60);
	stage = 55;
	endAuto = true;
	}
	else{//Waits to see if it will find a target again
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(60.0, 1.0), spin.getAngle());
	stage = 56;
	}
	
	
	}
	else if (backOff == true){//Backoff will run after 5 seconds of gear being gone
	stage = 6;
	if (backOffTime < 6.0){//Backs up off of peg
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(60.0, 0.75), spin.getAngle()-60);
	}
	else if (backOffTime < 9.5){//Drives forward for set distance, will get cut short by time probably
	driveBase.mecanumDrive_Cartesian(0.0, -0.8, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else {
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, spin.getAngle());
	}
	}
	else if (gearLifted == true){
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, spin.getAngle()-60);
	stage = 7;
	}
	}
	break;
	case customAuto3:// Gear right
	gearLifted = false;
	// ROTATION VALUES CORRECTED - UNSURE OF DIRECTIONAL CONTROL
	if (autoTime < 1.0){//Slow start driving, will always run at the start of the match
	driveBase.mecanumDrive_Cartesian(0.0, -0.5*autoTime, pointtoRotate(0.0, 1.0), spin.getAngle());
	stage = 10;
	}
	else{
	//Logic for deciding if targets were missed or if they were found
	//Will assume targets are missed if they are not found by set time
	//Will start dead reckoning once assumed, but can always switch to
	//Vision based align if targets are found. 
	if (contours == 0 && targetMissed == false && targetFound == false){
	if (autoTime > 4.5){
	targetMissed = true;
	stage = 11;
	}
	else{//Should drive past the peg if no targets are found
	driveBase.mecanumDrive_Cartesian(0.0, -0.38, pointtoRotate(300.0, 0.25), spin.getAngle());
	stage = 12;
	}
	}
	else if (contours != 0 && (gyroCorrected() < 297.0 || gyroCorrected() > 303.0 || autoTime < 2.5)){// Prevents code from thinking target is found at unreallistic times
	driveBase.mecanumDrive_Cartesian(0.0, -0.38, pointtoRotate(300.0, 0.25), spin.getAngle());
	stage = 13;
	}
	else if (contours != 0){
	targetFound = true;
	stage = 14;
	}
	
	//Timer continue to reset unless gear is lifted, so time will stay low unless gear is lifted
	//If gear is dropped timer will continue to reset
	if (gearLifted == false){
	backOffTimer.reset();
	}
	// After set time robot will start backing off
	if (backOffTime > 5.0){
	backOff = true;
	}
	else{
	backOff = false;
	}
	
	//Dead reckoning phase of auto. Back off phase has top priority. Will continue running if no return is triggered
	//despite if a target is found after the point of no return
	if ((targetMissed == true && targetFound == false || noReturn == true) && backOff == false && gearLifted == false){
	stage = 4;
	if (auto2Start == false){//Resets/starts timer once at first time if is executed
	autoTimer2.reset();
	auto2Start = true;
	stage = 41;
	}
	if (autoTime2 < 0.5){// Drives backwards to get in front of peg again
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 42;
	}
	else if (autoTime2 < 1.5){// Drives backwards to get in front of peg again
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 42;
	}
	else if (autoTime2 < 2.8){// Drives onto the peg, at this time if targets are found it wont matter, too close to peg
	noReturn = true;
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 43;
	}
	else if (autoTime2 < 7.0){// Waits set time at peg, will back off any time if gear is detected to be picked up
	driveBase.mecanumDrive_Cartesian(0.0, -0.125, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 44;
	}
	// Will back up no matter what in the last second of auto
	//Multiple thoughts, dead reckoning is assumed to have missed, if so backing up saves that time during tele
	//If dead reckoning hits, will save time to moving that distance in tele
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 45;
	endAuto = true;
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 46;
	}
	}//Will run vision based alignment unless robot starts backing up or dead reckoning is at no return point
	else if (targetFound == true && backOff == false && noReturn == false && gearLifted == false){
	stage = 5;
	if (contours == 1 && visionStart == false && endAuto == false){//Continues driving at a slower speed until two targets are found
	if(XDif < 0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.22, pointtoRotate(300.0, 1.0), spin.getAngle());
	}
	else if(XDif > 0){
	driveBase.mecanumDrive_Cartesian(0.0, -0.22, pointtoRotate(300.0, 1.0), spin.getAngle());
	}
	stage = 51;
	}
	if (contours == 2 && XDif > 6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.0, -0.22, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 52;
	}
	else if (contours == 2 && XDif < -6 && visionStart == false && endAuto == false){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.0, 0.22, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 53;
	}
	else if ((contours == 2 && Math.abs(XDif) < 6) || visionStart == true){
	if (autoStart == false){//Resets/starts timer once at first time if is executed
	autoTimer2.reset();
	autoStart = true;
	visionStart = true;//Flag will continue to run this if after it is executed once
	stage = 541;
	}
	else if (autoTime2 < 0.5){
	if (XDif > 3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(-0.09, 0.0, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 542;
	}
	else if (XDif < -3){//Aligns with peg
	driveBase.mecanumDrive_Cartesian(0.09, 0.0, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 543;
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, -0.0, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 544;
	}
	}
	
	else if (autoTime2 < 1.5){// Drives onto the peg, at this time if targets are found it wont matter, too close to peg
	if(contours == 2){
	driveBase.mecanumDrive_Cartesian((XDif/720), -0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	}
	else{
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	}
	stage = 545;
	}
	else if (autoTime2 < 2.5 && contours == 2){
	driveBase.mecanumDrive_Cartesian(0.0, -0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 546;
	}
	else if (autoTime2 < 7.0){// Waits set time at peg, will back off any time if gear is detected to be picked up
	driveBase.mecanumDrive_Cartesian(0.0, -0.125, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 547;
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 548;
	endAuto = true;
	}
	}
	else if (autoTime > 14.0 && autoTime < 15.0){
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 1.0), spin.getAngle()-300);
	stage = 55;
	endAuto = true;
	}
	else{//Waits to see if it will find a target again
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(300.0, 1.0), spin.getAngle());
	stage = 56;
	}
	
	
	}
	else if (backOff == true){//Backoff will run after 5 seconds of gear being gone
	stage = 6;
	if (backOffTime < 6.0){//Backs up off of peg
	driveBase.mecanumDrive_Cartesian(0.0, 0.4, pointtoRotate(300.0, 0.75), spin.getAngle()-300);
	}
	else if (backOffTime < 9.5){//Drives forward for set distance, will get cut short by time probably
	driveBase.mecanumDrive_Cartesian(0.0, -0.8, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	else {
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, spin.getAngle());
	}
	}
	else if (gearLifted == true){
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, spin.getAngle()-60);
	stage = 7;
	}
	}
	break;
	case defaultAuto:
	if (autoTime < 15){
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, pointtoRotate(0.0, 1.0), spin.getAngle());
	}
	case customAuto4://Short
	if (autoTime < 1.0){
	driveBase.mecanumDrive_Cartesian(0.0, -0.5*autoTime, 0.0, spin.getAngle());
	}
	else if (autoTime < 4.5){
	driveBase.mecanumDrive_Cartesian(0.0, -0.3, 0.0, spin.getAngle());
	}
	else {
	driveBase.mecanumDrive_Cartesian(0.0, 0.0, 0.0, spin.getAngle());
	}
	break;
	default:
	// Put default auto code here
	break;
	}
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	LED.set(Relay.Value.kOn);
	speedMode = speedChooser.getSelected();
	driveMode = driveChooser.getSelected();
	driveX = driveDirection.getX(); 
	driveY = driveDirection.getY();
	
	// Robot XY Speed Control Logic
	if (driveDirection.getRawButton(1)){
	driveX *= PRECISION_SPEED;
	driveY *= PRECISION_SPEED;
	speedModeSelected = "Precision";
	
	
	}
	else if (speedMode == speedModeMedium){
	driveX *= MEDIUM_SPEED;
	driveY *= MEDIUM_SPEED;
	speedModeSelected = "Medium";
	}
	else{
	speedModeSelected = "High";
	
	}
	
	//Deadband for XY controls
	if (Math.abs(driveX) < 0.03) {
	driveX = 0;
	}
	if (Math.abs(driveY) < 0.03) {
	driveY = 0;
	}
	
	//Set axis left right controls
	if (operator.getRawAxis(2) > 0.1){
	driveX = acceldriveStraight(0.0, 1.0); 
	}
	else if (driveDirection.getRawButton(5) || operator.getRawButton(6)){
	driveX = 0.35;
	}
	else if (driveDirection.getRawButton(4) || operator.getRawButton(5)){
	driveX = -0.35;
	}
	
	driveBase.mecanumDrive_Cartesian(driveX, driveY, rotationChooser(), xyMode());
	
	// Reset Gyro Heading
	if (driveRotation.getRawButton(2)){
	spin.reset();
	}
	
	// Relay control for lift mechanism	
	if (operator.getRawButton(7)){
	Lift.set(0.5);
	}
	
	else if (operator.getRawAxis(3)> 0.1){
	Lift.set(-operator.getRawAxis(3));
	}
	else if (driveRotation.getRawButton(3)){
	Lift.set(-driveRotation.getMagnitude());
	}
	else{
	Lift.set(0.0);
	}
	
	// Call (albeit unreliable) current match time; good to have on SmartDash
	double matchTime = DS.getMatchTime();	
	int matchSecs = ((int)matchTime%60);
	int matchMins = (int)(matchTime/60);
	String matchTimeString = matchMins + ":" + matchSecs;
	SmartDashboard.putString("Match Time", matchTimeString);
	SmartDashboard.putNumber("Value of X: ", driveX);
	SmartDashboard.putNumber("Value of Y: ", driveY);
	SmartDashboard.putString("Drive Mode:", driveModeSelected);
	SmartDashboard.putString("Speed Mode:", speedModeSelected);
	
	//Point to Rotate Debugging tools
	SmartDashboard.putNumber("Joystick Angle:", rotationjoystickCorrected());
	SmartDashboard.putNumber("Current Heading: ", gyroCorrected());	
	SmartDashboard.putNumber("Rotation Difference", rotationDif);
	SmartDashboard.putNumber("Rotation Output:", rotationChooser());	
	SmartDashboard.putNumber("Joystick Magnitude", driveRotation.getMagnitude());
	
	//Vision Debugging Tools
	SmartDashboard.putNumber("VisionPegAlign", visionpegAlign(1.0));
	SmartDashboard.putNumber("XDif Debug", XDif);
	
	//Accel Debugging Tools
	acceldriveStraight(gyroCorrected(), 1.0);
	SmartDashboard.putNumber("Accel Counter", accelCounter);
	SmartDashboard.putNumber("Accelerometer Value Field", XValField);
	SmartDashboard.putNumber("Accelerometer Value", accel.getX());
	
	gearLifted =! gearTrigger.get();
	SmartDashboard.putBoolean("gearLifted", gearLifted);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override  
	public void testPeriodic() {
	}

	// called by mecanumDrive_Cartesian for how it handles XY Input
	public double xyMode(){
	double angle;
	
	// Outputs to mecanum drive angle depending on mode
	if(driveMode == driveModeRobot){//xBox left &  right  bumper
	angle = 0.0;
	driveModeSelected = "Robot Oriented";
	}
	else if(driveDirection.getRawButton(4) || driveDirection.getRawButton(5) || operator.getRawButton(6) || operator.getRawButton(5) || (operator.getRawAxis(2) > 0.1)){
	angle = 0.0;
	driveModeSelected = "Axis Lock";
	}
	else{
	angle = spin.getAngle();
	driveModeSelected = "Field Oriented";
	}
	return angle;
	}
	
	//Returns a value for the heading of the robot from 0 to 360 degrees, clockwise
	public double gyroCorrected(){
	return (((spin.getAngle()%360)+360)%360);
	}
	
	//Returns a value for the direction of the joystick from 0 to 360 degrees, clockwise
	public double rotationjoystickCorrected(){
	return ((driveRotation.getDirectionDegrees()+360)%360);
	}
	
//Called during teleop which sets the value of rotation depending on which mode is used by driver
	public double rotationChooser(){
	double rotationSmart = 0;	//Value returned by function
	
	//Low rotation speed buttons used for turning robot to "North" heading to reset gyro value when needed
	if (driveRotation.getRawButton(5)){
	rotationSmart = 0.20;
	}
	else if (driveRotation.getRawButton(4)){
	rotationSmart = -0.20;
	}
	
	//Preset airship peg angles
	else if (operator.getRawButton(3)){
	rotationSmart = pointtoRotate(60.0, 1);
	}
	else if (operator.getRawButton(1)){
	rotationSmart = pointtoRotate(0.0, 1);
	}
	else if (operator.getRawButton(2)){
	rotationSmart = pointtoRotate(300.0, 1);
	}
	
	else{
	//When driver is in point to rotate mode
	if (driveRotation.getTrigger()){
	// Deadband to prevent jumping at low magnitude
	if (driveRotation.getMagnitude() < 0.05 && driveRotation.getMagnitude() > -0.05){
	rotationSmart = 0.0;
	}
	else{//Calls point to rotate using joystick angle for desired angle and joystick magnitude for magnitude
	rotationSmart = pointtoRotate(rotationjoystickCorrected(),((driveRotation.getMagnitude()/5)+0.8) * (((2 - Math.abs(driveY) - Math.abs(driveX)) / 2)/2 + 0.75));
	}
	}
	//Rotates based off normal rotation joystick x-magnitude
	else{
	// 50% Deadband to prevent jumping while going from PTR mode
	if(Math.abs(driveRotation.getX()) < 0.5){
	rotationSmart = 0;
	}
	else if (driveRotation.getX() <= -0.5){
	rotationSmart = 1.2*(driveRotation.getX() + 0.5);
	}
	else if (driveRotation.getX() >= 0.5){
	rotationSmart = 1.2*(driveRotation.getX() - 0.5);
	}
	else{
	rotationSmart = 0;
	}
	//Sets rotationDif as 0 to indicate on smartDashboard that the robot is not in PTR mode
	rotationDif = 0;
	}
	}
	if(driveRotation.getRawButton(7)){
	rotationSmart = driveRotation.getRawAxis(1);
	}
	if(driveRotation.getRawButton(3)){
	rotationSmart = 0.0;
	}
	
	return rotationSmart;
	}
	
	//Takes angle which the robot should point to and turns to that angle at speed controlled by magnitude
	public double pointtoRotate(double angle, double magnitude){
	//If the raw difference is greater than 180, which happens when the values cross to and from 0 * 360,
	//the value is subtracted by 360 to get the actual net difference
	if( (gyroCorrected() - angle) > 180){
	rotationDif = (gyroCorrected() - angle - 360) ;
	}
	else if( (gyroCorrected() - angle) < -180){
	rotationDif = (gyroCorrected() - angle + 360) ;
	}
	//If the magnitude of the difference is less than 180 than it is equal to the net difference. so nothing extra is done
	else{
	rotationDif = (gyroCorrected() - angle) ;
	}
	
	//Sets output rotation to inverted dif as a factor of the given magnitude
	//Uses cbrt to give greater output at mid to low differences
	double rotationPTR = 0.4 * Math.cbrt( rotationDif / -180 * magnitude);
	
	//Reduces rotation magnitude output is angle is within 4 degrees of desired
	if(Math.abs(rotationDif) < 4){
	rotationPTR = rotationDif / -180 * magnitude;
	}
	return rotationPTR;
	}
	public double visionpegAlign(double magnitude){
	double XAlign;
	double centerXAvg;
	double[] fallbackcenterXValue = new double [0];
	double total = 0.0;
	int element = 1;
	double[] centerX = table.getNumberArray("centerX", fallbackcenterXValue);
	contours = centerX.length;
	if (centerX.length < 1 /*&& DS.isAutonomous()*/) {
	centerXAvg = 160;
	}
	else {
	for (element = 0; element < centerX.length; element+=1){
	total += centerX[element];
	}
	centerXAvg = total / element;
	}
	
	XDif = centerXAvg - 160 ;
	
	XAlign = 0.33 * magnitude * XDif / 160;
	
	//if (Math.abs(XDif) < 10) {
	//XAlign = 0.23 * magnitude * (XDif / 160);
	//}
	SmartDashboard.putNumber("Center X Debug", centerXAvg);
	//SmartDashboard.putNumber("X Align Debug", XAlign);

	return XAlign;
	}
	
	public double acceldriveStraight(double angle, double magnitude){
	double XOutput, total = 0.0;
	double CCWAngle =  Math.abs((angle - 360)%360);
	XValField = (accel.getX()*Math.cos(Math.toRadians(CCWAngle))) - (accel.getY()*Math.sin(Math.toRadians(CCWAngle)));
	
	for (int i = 0; (i+1) < accelValues.length; i++){
	accelValues[i] = accelValues[i+1];
	}
	accelValues[accelValues.length-1] = XValField;
	accelCounter++;
	for (int i = 0; i < accelValues.length; i++){
	total += accelValues[i];
	}
	double average = total / accelValues.length;
	XOutput = -0.3 * average * magnitude;
	return XOutput;
	}
	
}
