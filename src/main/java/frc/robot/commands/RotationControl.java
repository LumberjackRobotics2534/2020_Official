/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotationControl extends CommandBase {
 /* ColorWheel m_colorWheel;
  double m_colorWheelMotorRotations;
  int m_colorWheelMotorPosition;
  public RotationControl(int colorWheelMotorPosition,double colorWheelMotorRotations,ColorWheel colorWheel) {
    m_colorWheelMotorRotations = colorWheelMotorRotations;
    m_colorWheel = colorWheel;
    m_colorWheelMotorPosition = colorWheelMotorPosition;
    addRequirements(m_colorWheel);*/
  //}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*m_colorWheelMotorPosition = ColorWheel.colorWheelMotor.getSelectedSensorPosition();
    m_colorWheelMotorRotations = m_colorWheelMotorPosition/Constants.quadrativeEncoderRoation;
    if(m_colorWheelMotorRotations<Constants.completedRotationNumber){
    RobotContainer.m_ColorWheel.spinColorWheel(Constants.colorWheelMotorSpeed);
    } else{
      RobotContainer.m_ColorWheel.spinColorWheel(0);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
