
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
  private WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.rightFrontDrive);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(Constants.rightBackDrive);
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.leftFrontDrive);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(Constants.leftBackDrive);
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
    leftBack.setSelectedSensorPosition(0);
    rightBack.setSelectedSensorPosition(0);
    navx.reset();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }
  //Mecanum drive command for manual control
  public void drive(double _ySpeed, double _xSpeed, double _rot){
    mecanumDriveTrain.driveCartesian(_ySpeed, _xSpeed, _rot);
  }

  public double getHeading() {
    return (-1)*Math.IEEEremainder(navx.getAngle(), 360);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(-rightVolts);
    System.out.println("left\t"+leftVolts);
    System.out.println("right\t"+(-rightVolts));
    System.out.println("angle\t" + getHeading());
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
      leftBack.getSelectedSensorVelocity()*Constants.kTicksToMetersConversion*Constants.kHundredMSToSecondsConversion,
      rightBack.getSelectedSensorVelocity()*Constants.kTicksToMetersConversion*Constants.kHundredMSToSecondsConversion);
  }

  @Override
  public void periodic() {
    //Update odometry with gyro values and encoder values
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      //Get distance travelled in TICKS, convert to METERS
      leftBack.getSelectedSensorPosition()*Constants.kTicksToMetersConversion,
      rightBack.getSelectedSensorPosition()*Constants.kTicksToMetersConversion * (-1));
    
  }
}
