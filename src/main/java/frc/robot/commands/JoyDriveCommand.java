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
import frc.robot.subsystems.DriveTrain;

public class JoyDriveCommand extends CommandBase {
  //Create varraibles for drive commands
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
    if (Math.abs(leftY) < 0.25) {
      leftY = 0;
    }
    if (Math.abs(leftX) < 0.25) {
      leftX = 0;
    }
    if (Math.abs(rightX) < 0.25) {
      rightX = 0;
    }
    m_DriveTrain.drive(leftY*Constants.driveSpeedScale, -leftX*Constants.driveSpeedScale, rightX*Constants.driveSpeedScale);
    /*System.out.print(leftY);
    System.out.print(-leftX);
    System.out.println(rightX);*/
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
