/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
  //Create Talons
  private WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.rightFrontDrive);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.rightBackDrive);
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.leftFrontDrive);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.leftBackDrive);
  //Create Mecanum drive for manual control
  private MecanumDrive mecanumDriveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  //Create Differentail drive for Motion Profiling
  private SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftBack);
  private SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightBack);
  private DifferentialDrive differentialDriveTrain = new DifferentialDrive(leftSide, rightSide);
  //Set up odomentry
  private DifferentialDriveOdometry m_odometry;
  //Set up gyro
  private AHRS navx = new AHRS(SerialPort.Port.kMXP);

  public DriveTrain() {
    //Configurations for talons
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setInverted(true);
     rightBack.setInverted(true);
     leftFront.setInverted(true);
      leftBack.setInverted(true);
    //Tells the robot if the encoders are reversed
    rightFront.setSensorPhase(false);
    leftFront.setSensorPhase(true);

    //Reset encoders before finishing creation of odometry
    leftFront.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
  }
  //Get gyro value and convert to be 180 to -180
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  //Mecanum drive command for manual control
  public void drive(double _ySpeed, double _xSpeed, double _rot){
    mecanumDriveTrain.driveCartesian(_ySpeed, _xSpeed, _rot);
  }
  //TankDrive command by setting exact voltage, bypassing .set() command becuase it automatically adjusts for voltage sag
  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(-rightVolts);
    differentialDriveTrain.feed();
  }

  @Override
  public void periodic() {
    //Update odometry with gyro values and encoder values
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      //Get distance travelled in TICKS, convert to METERS
      (leftFront.getSelectedSensorPosition()/241889.76378), //TODO re-calculate division
      (rightFront.getSelectedSensorPosition())/241889.76378);
  }
  public Pose2d getPose() {
    //Get position in METERS
    return m_odometry.getPoseMeters();
    
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //Get wheel speeds
    return new DifferentialDriveWheelSpeeds(
      //Get velocity in TICKS/100MS, convert to METERS/SECOND
      leftFront.getSelectedSensorVelocity() * (10/38497.9515889), //TODO re-calculate division
      rightFront.getSelectedSensorVelocity() * (10/38497.9515889));
  }
}
