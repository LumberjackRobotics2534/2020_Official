package frc.robot.commands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorWheel;

public class PositionControl extends CommandBase {
  String gameData;
  ColorWheel m_ColorWheel;
  ColorSensorV3 m_ColorSensor;
  boolean rotationsCompleted = false;
  public PositionControl(ColorWheel _colorWheel) {
      m_ColorWheel = _colorWheel;
      addRequirements(m_ColorWheel);
  }
  @Override
  public void initialize() {
    //Gets the color from the FMS thing and assigns that color to a variable
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    rotationsCompleted = false;
  }
  @Override
  public void execute() {
    if(gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'Y' :
        //yellow case code
        if (m_ColorWheel.getColor() == "G") {          
          //The wheel stays in place, it's already at the right color
          rotationsCompleted = true;
        }
        if (m_ColorWheel.getColor() == "Y") {
          //wheel rotates two tiles
          m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.25) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "R") {
            //need to do testing to see how long and at what power the motor needs to run
            /* **** SPIN THE MOTOR ONE TILE TO THE LEFT **** */
          m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "B") {
          //need to do testing to see how long and at what power the motor needs to run
          /* **** SPIN THE MOTOR ONE TITLE TO THE RIGHT **** */
          m_ColorWheel.spin(-Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        break;
        case 'R' :
        //red case code
        if (m_ColorWheel.getColor() == "Y") {
          //need to do testing to see how long and at what power the motor needs to run
          /* **** SPIN THE MOTOR ONE TITLE TO THE RIGHT **** */
          m_ColorWheel.spin(-Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "R") {
          //wheel rotates two tiles in any direction
          m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.25) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "G") {
          //need to do testing to see how long and at what power the motor needs to run
            /* **** SPIN THE MOTOR ONE TILE TO THE LEFT **** */
            m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "B") {
          //The wheel stays in place, it's already at the right color
          rotationsCompleted = true;
        }
        break;
        case 'G' :
        //green case code
        if (m_ColorWheel.getColor() == "Y") {
          //The wheel stays in place, it's already at the right color
          rotationsCompleted = true;
        }

        if (m_ColorWheel.getColor() == "G") {
          //wheel rotates two tiles
          m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.25) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "B") {
          //need to do testing to see how long and at what power the motor needs to run
            /* **** SPIN THE MOTOR ONE TILE TO THE LEFT **** */
            m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "R") {
            //need to do testing to see how long and at what power the motor needs to run
          /* **** SPIN THE MOTOR ONE TITLE TO THE RIGHT **** */
          m_ColorWheel.spin(-Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        break;
        case 'B' :
        //blue case code      
        if (m_ColorWheel.getColor() == "Y") {
          //need to do testing to see how long and at what power the motor needs to run
            /* **** SPIN THE MOTOR ONE TILE TO THE LEFT **** */
         m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "B") {
          //Wheel rotates two tiles
          m_ColorWheel.spin(Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.25) {
            rotationsCompleted = true;
          }
        }
        if (m_ColorWheel.getColor() == "R") {
          //The wheel stays in place, it's already at the right color
            rotationsCompleted = true;
        }
        if (m_ColorWheel.getColor() == "G") {
          //need to do testing to see how long and at what power the motor needs to run
            /* **** SPIN THE MOTOR ONE TILE TO THE RIGHT **** */
            m_ColorWheel.spin(-Constants.colorWheelPositionSpeed);
          if ( ColorWheel.getPosition() > 0.125) {
            rotationsCompleted = true;
          }
        }
        break;
        default :
        m_ColorWheel.spin(0);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_ColorWheel.spin(0);
  }

  @Override
  public boolean isFinished(){
    return rotationsCompleted;
  }
}