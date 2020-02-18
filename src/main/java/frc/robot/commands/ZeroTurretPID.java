/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ZeroTurretPID extends PIDCommand {
  /**
   * Creates a new ZeroTurretPID.
   */
  public ZeroTurretPID(Turret _Turret) {
    super(new PIDController(0.0075, 0, 0), _Turret::getPosition, 0,
        output -> {
          _Turret.spinTurret(output);
        },
        _Turret);
    getController().enableContinuousInput(Constants.zeroTurretMinimumInput, Constants.zeroTurretMaximumInput);
    getController().setTolerance(Constants.zeroTurretPositionTolerance, Constants.turretVelocityTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
