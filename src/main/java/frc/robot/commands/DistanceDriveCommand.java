/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

public class DistanceDriveCommand extends PIDCommand {
  public DistanceDriveCommand(DriveTrain _driveTrain, int _targetDistance) {
    super( 
        new PIDController(0, 0, 0),
        _driveTrain::getEncoderPosition,
        _targetDistance,
        output -> {
          _driveTrain.drive(output, 0, 0);
        },
        _driveTrain);
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
