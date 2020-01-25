/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.rightFrontDrive);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.rightBackDrive);
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.leftFrontDrive);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.leftBackDrive);
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  
  private MecanumDrive mecanumDriveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  private SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  private SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);
  private DifferentialDrive differentialDriveTrain = new DifferentialDrive(left, right);
  private DifferentialDriveOdometry m_odometry;

  public DriveTrain() {
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();

    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);

    rightFront.setInverted(false);
    rightBack.setInverted(false);
    leftFront.setInverted(false);
    leftBack.setInverted(false);
  }
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (1.0);
  }
  public void drive(double _ySpeed, double _xSpeed, double _rot){
    mecanumDriveTrain.driveCartesian(_ySpeed, _xSpeed, _rot);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts){
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    differentialDriveTrain.feed();
  }
  @Override
  public void periodic() {
    
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftFront.getSelectedSensorPosition(), rightFront.getSelectedSensorPosition());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
    
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity(), rightFront.getSelectedSensorVelocity());
  }
}
