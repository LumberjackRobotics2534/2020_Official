/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RGBstrip;

public class LEDCommand extends CommandBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx;
  public double x;

  RGBstrip m_ledstrip;
   public LEDCommand(RGBstrip _ledstrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ledstrip = _ledstrip;
    addRequirements(m_ledstrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = table.getEntry("tx");
    x = tx.getDouble(0.0);
    if(-2 <= x && x <= 2 && RobotContainer.manipButtonA.get()){
      for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
        m_ledstrip.m_LedBuffer.setRGB (i, 0, 255, 0);
      }

    } else if(-2 >= x && x >= 2 && RobotContainer.manipButtonA.get()){
      for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
        m_ledstrip.m_LedBuffer.setRGB (i, 128, 0, 128);
      }

    } else if(DriverStation.getInstance().isFMSAttached()){
      for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
        m_ledstrip.m_LedBuffer.setRGB (i, 0, 0, 255);
      }

    }else if(DriverStation.getInstance().isEnabled()){
      for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
        m_ledstrip.m_LedBuffer.setRGB (i, 255, 0, 0);
      }}
      else{
        for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
         m_ledstrip.m_LedBuffer.setRGB (i, 0, 0, 0);
        }

    }
      m_ledstrip.m_led.setData(m_ledstrip.m_LedBuffer);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (var i = 0; i < m_ledstrip.m_LedBuffer.getLength(); i++){
      m_ledstrip.m_LedBuffer.setRGB (i, 0, 0, 0);}
     
     m_ledstrip.m_led.setData(m_ledstrip.m_LedBuffer);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
