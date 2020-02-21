/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hang extends SubsystemBase {
  WPI_TalonSRX leftClimbMotor = new WPI_TalonSRX(Constants.leftClimbMotor);
  WPI_TalonSRX rightClimbMotor = new WPI_TalonSRX(Constants.rightClimbMotor);
  DoubleSolenoid climbSolenoid = new DoubleSolenoid(Constants.pcm, Constants.endGameUp, Constants.endGameDown);
  public Hang() {
    leftClimbMotor.configFactoryDefault();
    
    leftClimbMotor.setNeutralMode(NeutralMode.Brake);

    leftClimbMotor.setInverted(false);
    
    rightClimbMotor.configFactoryDefault();
  
    rightClimbMotor.setNeutralMode(NeutralMode.Brake);

    rightClimbMotor.setInverted(true);
  }
  public void Winch(double _speed){
    leftClimbMotor.set(_speed);
    rightClimbMotor.set(_speed);
  }
  public void Raise(){
    climbSolenoid.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
