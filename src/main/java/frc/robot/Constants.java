/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //-------------------DRIVE TRAIN-------------------//
	public static final int rightFrontDrive = 2;
	public static final int rightBackDrive = 4;
	public static final int leftFrontDrive = 1;
    public static final int leftBackDrive = 3;
    //----- -----------MOTION PROFILING----------------//
    public static final double ksVolts = 1.75;
    public static final double kvVoltSecondsPerMeter = 2.59;
    public static final double kaVoltSecondsSquaredPerMeter = 0.437;
    public static final double kPDriveVel = 15.3;
    public static final double kTrackwidthMeters = 0.61;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    //-------------------CONTROLLER-------------------//
    //Controller IDs
    public static final int driverControllerPort = 0;
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
    public static final double deadzone = 0.0;
    //---------------------INTAKE---------------------//

    //--------------------ELEVATOR--------------------//

    //---------------------TURRET---------------------//

    //---------------------SHOOTER--------------------//
    public static final int shooterMotor = 0;//TODO Change to correct ID
    public static final int shooterEncoder = 0;//TODO Change to correct ID
    //-------------------COLOR WHEEL------------------//
    public static final int colorWheelMotor = 0;//TODO Change to correct ID
    public static final int quadrativeEncoderRoation = 1024;
	public static final double colorWheelMotorSpeed = 0;
    //----------------------HANG----------------------//

}
