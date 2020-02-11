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
    //--------------------ELEVATOR--------------------//
    public static final int topElevatorMotor = 9;
    public static final int bottomElevatorMotor = 10;
    public static final double LiftSpeed = 0.6;
    //---------------------TURRET---------------------//
    public static final int turretMotor = 7;
    public static final double turretPositionTolerance = 1;
    public static final double turretVelocityTolerance = 0;
    public static final double turretMinimumInput = -29.8;
    public static final double turretMaximumInput = 29.8;
    public static final double turretTargetAngle = 0;
    public static final double turretDeadzone = 0;
    //---------------------SHOOTER--------------------//
    public static final int shooterMotor = 5;
    public static final int upHood = 0;
    public static final int downHood = 1;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public final static Gains kGains_Velocit = new Gains( 0.24, 0.001, 20, /*was 1023*/950.0/7200.0,  300,  1.00);
    public static final int shooterSpeed = -4500;
    //-------------------COLOR WHEEL------------------//
    public static final int colorWheelMotor = 6;
    public static final int quadrativeEncoderRotation = 36864;
    public static final double completedRotationNumber = 0;
	public static final double colorWheelMotorSpeed = 0;
    //----------------------HANG----------------------//
	//-------------------PNEUMATICS-------------------//
	public static final int pcm = 19;
	

	

	
	
	
	
	

}