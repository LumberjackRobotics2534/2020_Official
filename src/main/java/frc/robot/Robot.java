/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveTrain m_DriveTrain;
  private RobotContainer m_robotContainer;
  private ShootCommand m_ShootCommand;
  double current;
  double current0;
  PowerDistributionPanel pdp = new PowerDistributionPanel(20);
  int rpm;
  private Command m_autonomousCommand;
  WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.rightFrontDrive);
  WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.leftFrontDrive);
 // private JoyDriveCommand m_JoyDriveCommand;
  /*double current;
  double current0;
  PowerDistributionPanel pdp = new PowerDistributionPanel(20);*/
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    current = pdp.getTotalCurrent();
    current0 = pdp.getCurrent(0);
    SmartDashboard.putNumber("Current", current0);
    rpm = Math.abs(Shooter.shooterMotor.getSelectedSensorVelocity()*600/2048);
    SmartDashboard.putNumber("RPMs", rpm);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    //ColorWheel.colorWheelMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if(m_autonomousCommand != null){
      m_autonomousCommand.schedule();
      
    }
  }

  @Override
  public void teleopInit() {
  
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    
    
    rightFront.configFactoryDefault();
    leftFront.configFactoryDefault();

    rightFront.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);

    rightFront.setInverted(false);
    leftFront.setInverted(false);

    rightFront.setSensorPhase(false);
    leftFront.setSensorPhase(true);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    System.out.println(rightFront.getSelectedSensorPosition() + "right");
    System.out.println(leftFront.getSelectedSensorPosition() + "left");
  }
}
