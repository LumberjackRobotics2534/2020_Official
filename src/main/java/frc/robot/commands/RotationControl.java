/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorWheel;

public class RotationControl extends CommandBase {
  //Create varraibles for spin commands
  ColorWheel m_colorWheel;
  double m_colorWheelMotorRotations;
  double m_colorWheelMotorPosition;

  public RotationControl(ColorWheel colorWheel) {
    //Save inputs so we can use them later and add requirements
    m_colorWheel = colorWheel;
    addRequirements(m_colorWheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ColorWheel.colorWheelMotor.setSelectedSensorPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_colorWheelMotorPosition = ColorWheel.getPosition();
    if (m_colorWheelMotorRotations < Constants.completedRotationNumber){
    RobotContainer.m_ColorWheel.spin(Constants.colorWheelMotorSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_colorWheel.spin(0);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_colorWheelMotorRotations > Constants.completedRotationNumber){
      return true;
   } else{
      return false;
   }
 }
}
