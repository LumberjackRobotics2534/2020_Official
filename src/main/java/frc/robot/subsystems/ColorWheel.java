/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ColorWheel extends SubsystemBase {
  public static WPI_TalonSRX colorWheelMotor = new WPI_TalonSRX(Constants.colorWheelMotorID);
  /*private final I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);*/
  public ColorWheel() {

    /*colorWheelMotor.configFactoryDefault();
    
    colorWheelMotor.setNeutralMode(NeutralMode.Brake);

    colorWheelMotor.setInverted(false);*/
  }

  public void spinColorWheel(double speed) {

    colorWheelMotor.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
