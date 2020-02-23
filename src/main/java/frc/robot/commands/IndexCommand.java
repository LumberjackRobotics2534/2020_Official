/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;

public class IndexCommand extends CommandBase {
  private Elevator m_Elevator;
  private JoystickButton button;

  
  public IndexCommand(JoystickButton _button, Elevator _Elevator) {
    button = _button;
    m_Elevator = _Elevator;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Shooter.shooterReady){
      m_Elevator.lift();
    } else if (m_Elevator.topBallPresence()){
      m_Elevator.stopLifting();
    } else if(m_Elevator.bottomBallPresence()) {
      m_Elevator.lift();
    } else {
      m_Elevator.stopLifting();
    }
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
