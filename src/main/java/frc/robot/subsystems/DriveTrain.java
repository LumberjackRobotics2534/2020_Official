

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
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
    rightFront.setInverted(false);
    rightBack.setInverted(false);
    leftFront.setInverted(true);
    leftBack.setInverted(true);
    //Tells the robot if the encoders are reversed
    rightFront.setSensorPhase(true);
    leftFront.setSensorPhase(true);   
  }
  //Mecanum drive command for manual control
  public void drive(double _ySpeed, double _xSpeed, double _rot){
    mecanumDriveTrain.driveCartesian(_ySpeed, _xSpeed, _rot);
  }

  @Override
  public void periodic() {

  }
}
