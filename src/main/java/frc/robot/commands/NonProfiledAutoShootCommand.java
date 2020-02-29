/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class NonProfiledAutoShootCommand extends SequentialCommandGroup {
  Turret m_Turret;
  Shooter m_Shooter;
  int m_Balls;
  public NonProfiledAutoShootCommand(Turret _Turret, Shooter _Shooter, int Balls) {
    m_Turret = _Turret;
    m_Shooter = _Shooter;
    m_Balls = Balls;
    addCommands(
      new TurretPID(Constants.turretTargetAngle, m_Turret),
      new AutoShootCommand(m_Shooter, m_Balls)
    ); 
  }
}
