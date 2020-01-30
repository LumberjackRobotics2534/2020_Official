/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(Constants.elevatorMotorDrive);
  public Elevator() {
    elevatorMotor.configFactoryDefault();
    
    elevatorMotor.setNeutralMode(NeutralMode.Brake);

    elevatorMotor.setInverted(false);
  }
  public void liftBalls(double _liftSpeed){
    elevatorMotor.set(_liftSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
