/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AimTurretGroup extends SequentialCommandGroup {
  /**
   * Creates a new AimTurretCommand.
   */
  public AimTurretGroup(double _targetAngle, Turret _Turret, JoystickButton _Buton) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new WaitCommand(0.001),
      //new TurretPID(_targetAngle, _Turret, _Buton),
      //new TurretBangBang(_targetAngle, _Turret, Constants.BBTurretSpinSpeed * 3, Constants.BBTurretPositionTolerance * 10),
      new TurretBangBang(_targetAngle, _Turret, Constants.BBTurretSpinSpeed, Constants.BBTurretPositionTolerance)
    );
  }
}
