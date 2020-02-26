/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShootCommand extends CommandBase {
  private Shooter m_shooter;
  private JoystickButton m_Button;
  public static double dashRpm;
  private double equRpm;
  private double actualRpm;
  private double distance = 0.0;


  
  public ShootCommand(Shooter _shooter, JoystickButton _button) {
    m_shooter = _shooter;
    m_Button = _button;
    addRequirements(m_shooter); 
    SmartDashboard.putNumber("RPM", 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getDashVelocity();
    getEquationVelocity();
    if(m_Button.get()){
      double targetVelocity_UnitsPer100ms =  -dashRpm * 2048 / 600;//was -3387.5
      m_shooter.shoot(targetVelocity_UnitsPer100ms);
    }

    actualRpm = m_shooter.getAngularVelocity();
    SmartDashboard.putNumber("Actual RPM", actualRpm);

  }
  public void getDashVelocity(){
    dashRpm = SmartDashboard.getNumber("RPM", 0.0);
  }
  public void getEquationVelocity() {
    
    distance = Turret.getDistance();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_shooter.shooterOff();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
