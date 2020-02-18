/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.intakeMotorDrive);
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(Constants.intakeDown, Constants.intakeUp);
  public Intake() {

  }

public void intake(double spinSpeed) {
  intakeMotor.set(spinSpeed);
  intakeSolenoid.set(Value.kForward);
  }
public void stopIntake(){
  intakeMotor.set(0);
  intakeSolenoid.set(Value.kReverse);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
