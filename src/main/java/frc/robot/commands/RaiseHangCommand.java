/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Hang;

public class RaiseHangCommand extends CommandBase {
  private Hang m_Hang;
  private boolean finished = false;
  private JoystickButton m_Button;

  public RaiseHangCommand(Hang _Hang, JoystickButton _button) {
    m_Hang = _Hang;
    m_Button = _button;
    addRequirements(m_Hang);
    m_Hang.resetEncoder();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Hang.resetEncoder();
    finished = false;
    m_Hang.enableCompressor(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Button.get()){
      m_Hang.raise();
    }
    
    if (m_Hang.getWinchPosition() < Constants.winchRaiseDistance){
      m_Hang.winch();
    } else {
      m_Hang.stopWinch();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Hang.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
