/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.shooterMotor);
  double rpm;
  public Shooter() {
    /*shooterMotor.configFactoryDefault();
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setInverted(false);
   
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    shooterMotor.setSensorPhase(true);
    
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);*/
  }
  public void shoot(JoystickButton button) {
    if (button.get()) {
			double targetVelocity_UnitsPer100ms =  -2375 * 2048 / 600;
      shooterMotor.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);   
    }
  }
  public void shooterOff(){
    shooterMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
