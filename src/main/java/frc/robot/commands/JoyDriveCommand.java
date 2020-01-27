/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class JoyDriveCommand extends CommandBase {
  DoubleSupplier m_leftY;
  DoubleSupplier  m_leftX;
  DoubleSupplier m_rightX;
  double leftY;
  double leftX;
  double rightX;
  DriveTrain m_DriveTrain;

  public JoyDriveCommand(DoubleSupplier _leftY, DoubleSupplier _leftX, DoubleSupplier _rightX, DriveTrain _dTrain) {
    m_leftY = _leftY;
    m_leftX = _leftX;
    m_rightX = _rightX;
    m_DriveTrain = _dTrain;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftY = m_leftY.getAsDouble();
    leftX = m_leftX.getAsDouble();
    rightX = m_rightX.getAsDouble();
    if (Math.abs(leftY) < Constants.deadzone){ 
      leftY = 0.0;
    }
    if (Math.abs(leftX) < Constants.deadzone){
      leftX = 0.0;
    }
    if (Math.abs(rightX) < Constants.deadzone){
      rightX = 0.0;
    } 
    RobotContainer.m_DriveTrain.drive(m_leftY.getAsDouble(), -m_leftX.getAsDouble(), m_rightX.getAsDouble());
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
