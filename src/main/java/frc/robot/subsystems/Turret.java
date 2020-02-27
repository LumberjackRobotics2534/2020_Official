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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Other.SampleSmoother;



public class Turret extends SubsystemBase {
  //Creates Turret Motor 
  public WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.turretMotor);
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx;
  public double x;
  static NetworkTableEntry ty;
  static double y;
  static double distance;
  private DigitalInput turretZeroSensor = new DigitalInput(Constants.turretZeroSensorPort);
  private boolean zero;
  private double turretPosition;
  private static int lightCode = 1;
  static SampleSmoother distanceSmoother = new SampleSmoother(5);

  public Turret() {
    turretMotor.configFactoryDefault();

    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretMotor.setInverted(true);
  }

  public void spinTurret(double _speed) {
    turretMotor.set(_speed);
  }

  public void turretOff() {
    turretMotor.set(0);

  }

  public double getPosition() {
    turretPosition = turretMotor.getSelectedSensorPosition();
    return turretPosition;
  }

  public void zeroEncoder() {
    zero = turretZeroSensor.get();
    if (zero) {
      turretMotor.setSelectedSensorPosition(0);
    }
  }

  public void stopTurret() {
    if (turretMotor.getSelectedSensorPosition() == Constants.maxRightTurretPosition && turretMotor.get() > 0) {
      turretMotor.set(0);
    } else if (turretMotor.getSelectedSensorPosition() == Constants.maxLeftTurretPosition && turretMotor.get() < 0) {
      turretMotor.set(0);
    }
  }

  public double getX() {
    lightsEnabled(true);
    tx = table.getEntry("tx");
    x = tx.getDouble(0.0);
    return x;
  }

  public static double getDistance() {
    lightsEnabled(true);
    ty = table.getEntry("ty");
    y = ty.getDouble(0.0);
    distanceSmoother.addSample((54 / Math.tan(Math.toRadians(22 + y))));
    distance = distanceSmoother.getAverage();
    SmartDashboard.putNumber("Distance", distance);

    return distance;
  }

  public static void lightsEnabled(boolean enabled) {
    if (enabled) {
      lightCode = 3; // On
    } else {
      lightCode = 1; //Off
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(lightCode);
  }

  @Override
  public void periodic() {
    this.getDistance();
    this.stopTurret();
    SmartDashboard.putNumber("Hey Dude", this.getPosition());
  }
}
