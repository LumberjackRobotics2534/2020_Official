/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ZeroTurretCommand extends CommandBase {
  Turret m_Turret;
  public ZeroTurretCommand(Turret _Turret) {
    m_Turret = _Turret;
    addRequirements(m_Turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Turret.turretZeroSensor.get()){ //false when zeroed
      if(-100 >= m_Turret.getPosition()){
        m_Turret.spinTurret(0.3);
      } else if(m_Turret.getPosition() >= 100){
       m_Turret.spinTurret(-0.3);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Turret.spinTurret(0);
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!m_Turret.turretZeroSensor.get()){
      return true;
    }else if(-100 <= m_Turret.getPosition() && m_Turret.getPosition() <= 100){
      return true;
    } else{
      return false;
    }
  }
}
