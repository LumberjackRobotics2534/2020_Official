/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

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
  public static boolean shooterReady = false;
  
  public ShootCommand(Shooter _shooter, JoystickButton _button) {
    m_shooter = _shooter;
    m_Button = _button;
    addRequirements(m_shooter); 
    //SmartDashboard.putNumber("RPM", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Turret.lightsEnabled(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //getDashVelocity();
    getEquationVelocity();
    if(m_Button.get()){
      double targetVelocity_UnitsPer100ms =  -equRpm* 2048 / 600;
      m_shooter.shoot(targetVelocity_UnitsPer100ms);
    }

    actualRpm = m_shooter.getAngularVelocity();
    //SmartDashboard.putNumber("Actual RPM", actualRpm);
    if (actualRpm > equRpm - equRpm*Constants.acceptableRpmError 
    && actualRpm < equRpm + equRpm*Constants.acceptableRpmError){
      shooterReady = true; 
    } else {
      shooterReady = false;
    }
  }

  public void getDashVelocity(){
    //dashRpm = SmartDashboard.getNumber("RPM", 0.0);
  }
  
  public void getEquationVelocity() {
    distance = Turret.getDistance();
    if(distance >= 67 && distance <= 127){
     equRpm = /*(0.0000422)*Math.pow(distance, 4)*-1 + 0.01756*Math.pow(distance, 3) + -2.706*Math.pow(distance, 2) + 183.9*Math.pow(distance, 1)*/ 4900 ;//was 2154
    } else if(distance > 127 && distance < 169){
     equRpm = ((Math.pow(distance - 127, 2))/12)+ 3547.728; // 2547.728
    } else if(distance >= 169 && distance <= 400){
     equRpm = (0.0000002801)*Math.pow(distance, 4) - 0.0002518*Math.pow(distance, 3) + 0.08358*Math.pow(distance, 2) - 12.15*Math.pow(distance, 1) + 5653;
    } else if(distance > 400 ){
     equRpm = 5600;
    } else{
     equRpm = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_shooter.shooterOff();
    shooterReady = false;
    Turret.lightsEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
