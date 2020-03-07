/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoCommand extends SequentialCommandGroup {
  Turret m_Turret;
  Shooter m_Shooter;
  DriveTrain m_driveTrain;
  double m_Speed;
  public ThreeBallAutoCommand(Turret _Turret, Shooter _Shooter, DriveTrain _driveTrain, double _Speed, JoystickButton button) {
    m_Turret = _Turret;
    m_Shooter = _Shooter;
    m_driveTrain = _driveTrain;
    m_Speed = _Speed;
    addCommands(
      new NonProfiledAutoShootCommand(m_Turret, m_Shooter, 3, button),
      new WaitCommand(.5),
      new DistanceDriveCommand(m_driveTrain, m_Speed, 14000)
    );
  }
}
