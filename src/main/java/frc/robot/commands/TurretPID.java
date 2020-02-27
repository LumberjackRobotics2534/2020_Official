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
public class TurretPID extends PIDCommand {
  public TurretPID(double _targetAngle, Turret _Turret) {
    super(new PIDController(0.0075, 0, 0),_Turret::getX, _targetAngle,
      output -> {
        _Turret.spinTurret(output);
      },
      _Turret);
    
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(Constants.turretMinimumInput, Constants.turretMaximumInput);
    getController().setTolerance(Constants.turretPositionTolerance, Constants.turretVelocityTolerance);
    //SmartDashboard.putNumber("Skew", x);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Turret.lightsEnabled(false);
    return getController().atSetpoint();

  }
}
