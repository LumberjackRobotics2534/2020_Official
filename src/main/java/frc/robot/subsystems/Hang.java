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
  WPI_TalonSRX climbMotor = new WPI_TalonSRX(Constants.climbMotor);
  DoubleSolenoid climbSolenoid = new DoubleSolenoid(Constants.pcm, Constants.endGameDown, Constants.endGameUp);
  public Hang() {
    climbMotor.configFactoryDefault();
  
    climbMotor.setNeutralMode(NeutralMode.Brake);

    climbMotor.setInverted(false);
  }
  public void Winch(){
    climbMotor.set(0.75);
  }
  public void StopWinch(){
    climbMotor.set(0);
  }
  public void Raise(){
    climbSolenoid.set(Value.kForward);
  }
  public void Lower(){
    climbSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
