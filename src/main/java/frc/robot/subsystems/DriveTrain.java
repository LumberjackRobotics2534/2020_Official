
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //Create Talons
  private WPI_TalonFX rightFront = new WPI_TalonFX(Constants.rightFrontDrive);
  private WPI_TalonFX rightBack = new WPI_TalonFX(Constants.rightBackDrive);
  private WPI_TalonFX leftFront = new WPI_TalonFX(Constants.leftFrontDrive);
  private WPI_TalonFX leftBack = new WPI_TalonFX(Constants.leftBackDrive);
  //Create Mecanum drive for manual control
  private MecanumDrive mecanumDriveTrain = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  private SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftBack);
  private SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightBack);
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
    //Reset encoders before finishing creation of odometry
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    navx.zeroYaw();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }
  //Mecanum drive command for manual control
  public void drive(double _ySpeed, double _xSpeed, double _rot){
    mecanumDriveTrain.driveCartesian(_ySpeed, _xSpeed, _rot);
  }

  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(/*-*/rightVolts);
    System.out.println("left\t"+leftVolts);
    System.out.println("right\t"+rightVolts);
    mecanumDriveTrain.feed();
  }
  
  public Pose2d getPose() {
    //Get position in METERS
    System.out.println("pose\t" + m_odometry.getPoseMeters());
    return m_odometry.getPoseMeters();

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //Get wheel speeds
    return new DifferentialDriveWheelSpeeds(
      //Get velocity in TICKS/100MS, convert to METERS/SECOND
      leftFront.getSelectedSensorVelocity()*Constants.kTicksToMetersConversion*Constants.kHundredMSToSecondsConversion,
      rightFront.getSelectedSensorVelocity()*Constants.kTicksToMetersConversion*Constants.kHundredMSToSecondsConversion);
  }

  @Override
  public void periodic() {
    //Update odometry with gyro values and encoder values
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      //Get distance travelled in TICKS, convert to METERS
      leftFront.getSelectedSensorPosition()*Constants.kTicksToMetersConversion,
      rightFront.getSelectedSensorPosition()*Constants.kTicksToMetersConversion/* * (-1)*/);
    
  }
}
