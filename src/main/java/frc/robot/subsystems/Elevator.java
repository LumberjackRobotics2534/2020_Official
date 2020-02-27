/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private boolean topPresence = false;
  private boolean bottomPresence = false;
  private int bottomPresenceTicks = 0;

  private AnalogInput irTop = new AnalogInput(Constants.irSensorTop);
  private AnalogInput irBottom = new AnalogInput(Constants.irSensorBottom);
  private WPI_TalonSRX topElevatorMotor = new WPI_TalonSRX(Constants.topElevatorMotor);
  private WPI_TalonSRX bottomElevatorMotor = new WPI_TalonSRX(Constants.bottomElevatorMotor);
  public Elevator() {
    topElevatorMotor.configFactoryDefault();

    topElevatorMotor.setNeutralMode(NeutralMode.Brake);

    topElevatorMotor.setInverted(false);

    bottomElevatorMotor.configFactoryDefault();

    bottomElevatorMotor.setNeutralMode(NeutralMode.Brake);

    bottomElevatorMotor.setInverted(true);

    topPresence = false;
    
    bottomPresence = false;
  }

  public void lift() {
    topElevatorMotor.set(Constants.topLiftSpeed);
    bottomElevatorMotor.set(Constants.bottomLiftSpeed);
  }
  public void feed(){
    topElevatorMotor.set(Constants.topFeedSpeed);
    bottomElevatorMotor.set(Constants.bottomFeedSpeed);
  }

  public void stopAll() {
    topElevatorMotor.set(0);
    bottomElevatorMotor.set(0);
  }

  public boolean topBallPresence() {
    if(irTop.getAverageVoltage() > Constants.minTopPresenceVoltage ){
      topPresence = true;
    }else{
      topPresence = false;
    }
    return(topPresence);
  }
  public boolean bottomBallPresence() {

    if(bottomPresenceTicks > Constants.minPresenceTicks){
      bottomPresence = true;
    }else if (irBottom.getAverageVoltage() > Constants.minBottomPresenceVoltage
           && irBottom.getAverageVoltage() < Constants.maxBottomPresenceVoltage) {
      bottomPresenceTicks++;
    } else {
      bottomPresenceTicks = 0;
      bottomPresence = false;
    }
    return(bottomPresence);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
