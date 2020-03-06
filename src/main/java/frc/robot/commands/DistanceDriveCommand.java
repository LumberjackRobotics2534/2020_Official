/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DistanceDriveCommand extends CommandBase {
  DriveTrain m_DriveTrain;
  double m_speed;
  int m_distance = 1000000000;
  public DistanceDriveCommand(DriveTrain _DriveTrain, double _speed, int _distance) {
    m_DriveTrain = _DriveTrain;
    m_speed = _speed;
    m_distance = _distance;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveTrain.zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_DriveTrain.drive(0, m_speed, m_speed*.02333);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveTrain.zero();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_DriveTrain.getEncoderPosition()) > m_distance){
      System.out.println(Math.abs(m_DriveTrain.getEncoderPosition()) + "true");

      return true;
    } else{
      System.out.println(Math.abs(m_DriveTrain.getEncoderPosition()));

      return false;
    }
     
  }
}
