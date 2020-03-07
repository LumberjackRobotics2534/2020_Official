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
  private WPI_TalonSRX climbMotor = new WPI_TalonSRX(Constants.climbMotor);
  private DoubleSolenoid climbSolenoid = new DoubleSolenoid(Constants.pcm, Constants.endGameUp, Constants.endGameDown);
  private double winchPosition = 0;
  public Hang() {
    climbMotor.configFactoryDefault();
  
    climbMotor.setNeutralMode(NeutralMode.Brake);

    climbMotor.setInverted(true);

    climbMotor.setSensorPhase(true);

    lower();
    resetEncoder();
  }
  public void winch(){
    climbMotor.set(0.6);
  }
  public void winchBackwards() {
    climbMotor.set(-0.6);
  }
  public void stopWinch(){
    climbMotor.set(0);
  }
  public void raise(){
    climbSolenoid.set(Value.kReverse);
  }
  public void lower(){
    climbSolenoid.set(Value.kForward);
  }
  public double getWinchPosition(){
    winchPosition = -climbMotor.getSelectedSensorPosition();
    return winchPosition;
  }
  public void resetEncoder(){
    climbMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    getWinchPosition();
    // This method will be called once per scheduler run
  }
}
