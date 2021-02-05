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
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
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
  public DigitalInput turretZeroSensor = new DigitalInput(Constants.turretZeroSensorPort);
  private boolean zero = false;
  private double turretPosition;
  private static int lightCode = 1;
  static SampleSmoother distanceSmoother = new SampleSmoother(10);
  private static Relay flashlight = new Relay(Constants.flashlightRelay);

  public Turret() {
    turretMotor.configFactoryDefault();

    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretMotor.setInverted(true);

    zero = false;
  }

  public void spinTurret(double _speed) { // POS value is LEFT
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
    // System.out.println(zero);
    if (zero == false) {
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
    tx = table.getEntry("tx");
    x = tx.getDouble(0.0);
    return x;
  }

  public static double getDistance() {
    ty = table.getEntry("ty");
    y = ty.getDouble(0.0);
    distanceSmoother.addSample((54 / Math.tan(Math.toRadians(22 + y))));
    distance = distanceSmoother.getAverage();
    return distance;
  }

  public static void lightsEnabled(boolean enabled) {
    if (enabled) {
      lightCode = 3; // On
      flashlight.set(Value.kForward);
    } else {
      lightCode = 1; //Off
      flashlight.set(Value.kReverse);
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(lightCode);
  }

  @Override
  public void periodic() {
    stopTurret();
    getPosition();
    zeroEncoder();
    System.out.println(getDistance());
    flashlight.get();
    //System.out.println(turretPosition);
    //System.out.println(zero);
    lightsEnabled(true);
  }
}
