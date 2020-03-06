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
import frc.robot.subsystems.Turret;

public class TurretCommand extends CommandBase {
  DoubleSupplier m_Speed;
  Turret m_Turret;
  public TurretCommand(DoubleSupplier speed, Turret turret) {
    m_Speed = speed;
    m_Turret = turret;
    addRequirements(m_Turret);
  }

  
  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
  double speed = m_Speed.getAsDouble();
    if (Math.abs(speed) < Constants.turretDeadzone) {
      speed = 0.0;
    }
    RobotContainer.m_Turret.spinTurret(speed*-.3);
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
