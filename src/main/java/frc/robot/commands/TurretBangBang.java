/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurretBangBang extends CommandBase {
  private Turret m_Turret;
  private boolean finished;
  private double targetAngle;

  public TurretBangBang(double _targetAngle, Turret _Turret) {
    m_Turret = _Turret;
    targetAngle = _targetAngle;
    addRequirements(m_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Turret.getX() > 0 && m_Turret.getX() > Constants.BBTurretPositionTolerance){
      m_Turret.spinTurret(-Constants.BBTurretSpinSpeed); //TODO: Check that motor inversions for this method are correct
    } else if (m_Turret.getX() < 0 && m_Turret.getX() < Constants.BBTurretPositionTolerance){
      m_Turret.spinTurret(Constants.BBTurretSpinSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Turret.spinTurret(0);
    Turret.lightsEnabled(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_Turret.getX()) < Constants.BBTurretPositionTolerance && m_Turret.getX() != 0){
      System.out.println("BB FINISHED");
      return true;
    } else {
      System.out.println("BANG BANG");
      return false;
    }
  }
}
