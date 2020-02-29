/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Hang;

public class WinchCommand extends CommandBase {
  Hang m_Hang;
  JoystickButton m_Button1;
  JoystickButton m_Button2;
  JoystickButton m_Button3;
  public WinchCommand(Hang _Hang, JoystickButton _Button1, JoystickButton _Button2, JoystickButton _Button3) {
    m_Hang = _Hang;
    m_Button1 = _Button1;
    m_Button2 = _Button2;
    m_Button3 = _Button3;
    addRequirements(m_Hang);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Button1.get()){
      m_Hang.Winch();
    } else {
      m_Hang.StopWinch();
    }
    if(m_Button3.get()){
      m_Hang.WinchBackwards();
    }
    if (m_Button2.get()){
      m_Hang.Raise();
    } else {
      m_Hang.Lower();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Hang.StopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
