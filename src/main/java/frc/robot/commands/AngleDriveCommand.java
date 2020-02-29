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
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AngleDriveCommand extends PIDCommand {
  public AngleDriveCommand(DriveTrain _driveTrain, double _targetAngle) {
    super(
        new PIDController(0, 0, 0),
        _driveTrain::getHeading,
        _targetAngle,
        output -> {
          _driveTrain.drive(0, 0, output);
        },
        _driveTrain);
    getController().enableContinuousInput(Constants.angleDriveMinimumInput, Constants.angleDriveMaximumInput);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
