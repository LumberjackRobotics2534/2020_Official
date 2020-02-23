/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.Other.Gains;

public final class Constants {

    //-------------------DRIVE TRAIN-------------------//
	public static final int rightFrontDrive = 2;
	public static final int rightBackDrive = 4;
	public static final int leftFrontDrive = 1;
    public static final int leftBackDrive = 3;
    public static final int integratedEncoderTicksPerRot = 2048*(105/26);
    //----- -----------MOTION PROFILING----------------//
    public static final double ksVolts = 0.111;
    public static final double kvVoltSecondsPerMeter = 1.62;
    public static final double kaVoltSecondsSquaredPerMeter = 0.179;
    public static final double kMaxVoltage = 10;
    public static final double kPDriveVel = 7.6; //was 7.6
    public static final double kTrackwidthMeters = 0.5588; //was 1.1541452808922656
    public static final double kMaxSpeedMetersPerSecond = 3; //was 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //was 3
    public static final double kTicksToMetersConversion = ((1/14703.6897/*Rot per Tick*/)*(6*Math.PI/*In per Rot*/)*(1/39.3701/*M per In*/));
    public static final double kHundredMSToSecondsConversion = 10;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    //-------------------CONTROLLER-------------------//
    //Controller IDs
    public static final int driverControllerPort = 0;
    public static final int manipControllerPort = 1;
    //Controller Joystick IDs
    public static final int joyLeftX = 0;
    public static final int joyLeftY = 1;
    public static final int joyRightX = 4;
    //Controller Button IDs
    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
    public static final int buttonLeft = 5;
    public static final int buttonRight = 6;
    //Deadzone
    public static final double deadzone = 0.15;
    //---------------------INTAKE---------------------//
    public static final int intakeMotorDrive = 8;
    public static final double SpinSpeed = 0.8;
    public static final int intakeUp = 7;
    public static final int intakeDown = 6;
    //--------------------ELEVATOR--------------------//
    public static final int topElevatorMotor = 9;
    public static final int bottomElevatorMotor = 10;
    public static final double LiftSpeed = 0.6;
    public static final int irSensorTop = 2;
    public static final int irSensorBottom = 1;
    public static final double minTopPresenceVoltage = 1.5;
    public static final double minBottomPresenceVoltage = 1.25;
    public static final double maxBottomPresenceVoltage = 1.85;
    public static final double topLiftSpeed = 0.6;
    public static final double bottomLiftSpeed = 0.5;
    public static final int minPresenceTicks = 200; //TODO: Adjust value of ticks
    //---------------------TURRET---------------------//
    public static final int turretMotor = 7;
    public static final double turretPositionTolerance = 1;
    public static final double turretVelocityTolerance = 0;
    public static final double turretMinimumInput = -29.8;
    public static final double turretMaximumInput = 29.8;
    public static final double turretTargetAngle = 0;
    public static final double turretDeadzone = 0;
    public static final int turretZeroSensorPort = 0;
    public static final double zeroTurretMinimumInput = 0;
    public static final double zeroTurretMaximumInput = 0;
    public static final double zeroTurretPositionTolerance = 100;
    public static final double maxRightTurretPosition = 0;
	public static final double maxLeftTurretPosition = 0;
    //---------------------SHOOTER--------------------//
    public static final int shooterMotor = 5;
    public static final int upHood = 2;
    public static final int downHood = 3;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final double acceptableRpmError = 0.05; //5%
    public final static Gains kGains_Velocit = new Gains( 1.6, 0.004, 20, 1023/7200,  300,  1.00);
    public static final int shooterSpeed = -4500;
    //-------------------COLOR WHEEL------------------//
    public static final int colorWheelMotor = 6;
    public static final double quadrativeEncoderRotation = 1024*(36/*Reduction from Motor to WOF*/);
    public static final double completedRotationNumber = 26;
	public static final double colorWheelMotorSpeed = 0.5;
    //----------------------HANG----------------------//
    public static final int leftClimbMotor = 11;
    public static final int rightClimbMotor = 12;
    public static final int endGameUp = 4;
    public static final int endGameDown = 5;
    //-------------------PNEUMATICS-------------------//
    public static final int pressureSensor = 0;
    public static final int pcm = 19;
    //----------------------LEDS----------------------//
	public static final int ledPort = 9;
	


	
	

	

	
	
	
	
	

}