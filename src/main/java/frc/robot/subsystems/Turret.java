/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Turret extends SubsystemBase {
  //Creates Turret Motor 
  public WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.turretMotor);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx;
  public double x;
  DigitalInput turretZeroSensor = new DigitalInput(Constants.turretZeroSensorPort);
  boolean zero;
  double turretPosition;
  
  public Turret() {
    turretMotor.configFactoryDefault();
    
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretMotor.setInverted(false);
  }

public void spinTurret(double _speed){
  turretMotor.set(_speed);
}
public double getPosition(){
  turretPosition = turretMotor.getSelectedSensorPosition();  
  return turretPosition;
}
public void zeroEncoder(){
  zero = turretZeroSensor.get();
  if(zero){
    turretMotor.setSelectedSensorPosition(0);
  }
}
public void stopTurret(){
  if(turretMotor.getSelectedSensorPosition() == Constants.maxRightTurretPosition && turretMotor.get() > 0){
    turretMotor.set(0);
  } else if(turretMotor.getSelectedSensorPosition() == Constants.maxLeftTurretPosition && turretMotor.get() < 0){
    turretMotor.set(0);
  }
}
public double getX() {
  tx = table.getEntry("tx");
  x = tx.getDouble(0.0);
  return x;
}
  @Override
  public void periodic() {
    this.zeroEncoder();
    this.stopTurret();
  }
}
