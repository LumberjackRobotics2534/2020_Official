/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.shooterMotor);
  public static DoubleSolenoid hoodSolenoid = new DoubleSolenoid(Constants.pcm, Constants.upHood, Constants.downHood);

  public Shooter() {
    shooterMotor.configFactoryDefault();
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setInverted(false);
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
  }

  public void shoot(double targetVelocity) {
    shooterMotor.set(ControlMode.Velocity, targetVelocity);
  }

  public void shooterOff(){
    shooterMotor.set(0);
  }
  
  public void hoodUp(){
    hoodSolenoid.set(Value.kReverse);
  }
  
  public void hoodDown(){
    hoodSolenoid.set(Value.kForward);
  }
  public double getAngularVelocity(){
    return -shooterMotor.getSelectedSensorVelocity()*600 / 2048;
  }


  @Override
  public void periodic() {
    
  }
}
