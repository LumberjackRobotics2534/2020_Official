/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretPID extends PIDCommand {
  private Turret m_Turret;
  private boolean finished = false;
  private JoystickButton m_Button;
  /*private static double P = 0.0;
  private static double I = 0.0;
  private static double D = 0.0;*/

  public TurretPID(double _targetAngle, Turret _Turret, JoystickButton _Buton) {

    super(new PIDController(0.010, 0, 0), // 0.0105, 0.0058, 0.0001 //0.012, 0.0006, 0.0001, 0
      _Turret::getX,
      _targetAngle,
      output -> {
        _Turret.spinTurret(output);
      },
      _Turret);
    m_Turret = _Turret;
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(Constants.turretMinimumInput, Constants.turretMaximumInput);
    getController().setTolerance(Constants.PIDTurretPositionTolerance, Constants.turretVelocityTolerance);
    m_Button = _Buton;
  }
  @Override
  public void initialize() {
    Turret.lightsEnabled(true);
    /*SmartDashboard.putNumber("P", 0.0085);
    SmartDashboard.putNumber("I", 0.0058);
    SmartDashboard.putNumber("D", 0.0001);*/
  }



  @Override
  public void end(boolean finished){
    finished = false;
    Turret.lightsEnabled(false);

  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    finished = false;
    if(Math.abs(m_Turret.getX()) <= Constants.PIDTurretPositionTolerance && Math.abs(m_Turret.getX()) != 0.0){
      finished = true;
    } else{
      finished = false;
    }
    System.out.println("PID");
    return finished;
  }
  
}
