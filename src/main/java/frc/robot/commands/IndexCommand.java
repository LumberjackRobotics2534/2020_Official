/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class IndexCommand extends CommandBase {
  private Elevator m_Elevator;
  private boolean wasFeeding = false;

  public IndexCommand(Elevator _Elevator) {
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
    if (AutoShootCommand.shooterReady) {
      m_Elevator.feed();
    } else if (m_Elevator.topBallPresence()) {
      m_Elevator.stopAll();
    } else if (m_Elevator.bottomBallPresence()) {
      m_Elevator.lift();
      m_Elevator.reset();
      wasFeeding = true;
    } else {
      if (wasFeeding){
        m_Elevator.lift();
        if (m_Elevator.getDistance() > Constants.extraFeedDistance){
          m_Elevator.stopAll();
          wasFeeding = false;
        }
      }else{
        m_Elevator.stopAll();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.stopAll();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
