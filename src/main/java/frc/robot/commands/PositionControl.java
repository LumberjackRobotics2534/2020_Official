/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorWheel;


public class PositionControl extends CommandBase {

  String gameData;
  ColorWheel m_ColorWheel;
  ColorSensorV3 m_ColorSensor;
  int yellow1;
  int yellow2;
  int red1;
  int red2;
  int green1;
  int green2;
  int blue1;
  int blue2;


  /**
   * Creates a new PositionControl.
   */
  public PositionControl(ColorWheel _colorWheel, ColorSensorV3 _colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_ColorWheel = _colorWheel;
      addRequirements(m_ColorWheel);
      m_ColorSensor = _colorSensor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Gets the color from the FMS thing and assigns that color to a variable

    gameData = DriverStation.getInstance().getGameSpecificMessage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(gameData.length() > 0) {

      switch (gameData.charAt(0)) {

        case 'Y' :
        //yellow case code

        if (m_ColorWheel.getColor() == "G") {
          return;
        }

        if (m_ColorWheel.getColor() == "R") {

            

        }
        
        while (m_ColorWheel.getColor() != "Y") {
          m_ColorWheel.spin(.1);
        }
        break;

        case 'R' :
        //red case code

        while (m_ColorWheel.getColor() != "R") {
          m_ColorWheel.spin(.1);
        }
        break;

        case 'G' :
        //green case code

        while (m_ColorWheel.getColor() != "G") {
          m_ColorWheel.spin(.1);

        }
        break;

        case 'B' :
        //blue case code
        
        while (m_ColorWheel.getColor() != "B") {
          m_ColorWheel.spin(.1);

        }

        break;

        default :

        m_ColorWheel.spin(0);

      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public void spinUntil(float _encoderUnits, float _speed) {

    colorWheelMotor.setSelectedSensorPosition(0);

    while (/*encoder object value*/ <= _encoderUnits) {

      colorWheelMotor.spin(1);

    }

  }
}
