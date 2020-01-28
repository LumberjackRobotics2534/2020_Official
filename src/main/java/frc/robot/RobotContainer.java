package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.commands.JoyDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TurretCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController driverController = new XboxController(Constants.driverControllerPort);
  public static XboxController manipController = new XboxController(Constants.manipControllerPort);
  public static final DriveTrain m_DriveTrain = new DriveTrain();
  public static final ColorWheel m_ColorWheel = new ColorWheel();
  public static final Shooter m_Shooter = new Shooter();
  public static final Turret m_Turret = new Turret();
  JoystickButton buttonA = new JoystickButton(driverController, Constants.buttonA);
  JoystickButton buttonB = new JoystickButton(driverController, Constants.buttonB);
  JoystickButton buttonX = new JoystickButton(driverController, Constants.buttonX);
  JoystickButton buttonY = new JoystickButton(driverController, Constants.buttonY);
  JoystickButton buttonLeft = new JoystickButton(driverController, Constants.buttonLeft);
  JoystickButton buttonRight = new JoystickButton(driverController, Constants.buttonRight);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_DriveTrain.setDefaultCommand(new JoyDriveCommand(
        () -> driverController.getX(Hand.kLeft),
        () -> driverController.getY(Hand.kLeft), 
        () -> driverController.getX(Hand.kRight), m_DriveTrain));
        //Sets TurretCommand as Default Command for the Turret Subsystem
     m_Turret.setDefaultCommand(new TurretCommand(
        //Passes in the Left Joysick X value
        () -> manipController.getX(Hand.kLeft),m_Turret));
  }

   
  private void configureButtonBindings() {
    buttonA.whenHeld(new ShootCommand(m_Shooter, buttonA));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      Constants.kMaxVoltage); //This is the max voltage
    //Create a trajectory configuration
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(Constants.kDriveKinematics)
      // Apply the voltage constraint
      .addConstraint(autoVoltageConstraint);
    //Create a Trajectory to follow. UNITS ARE IN METERS from starting position.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      //Start at the origin facing the positive X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      //Pass through these interior points
      List.of(/*new Translation2d(1, 1), new Translation2d(2, -1)*/),
      //End position
      new Pose2d(3, 0, new Rotation2d(0)),
      //Pass config
      config);
    //Create RamseteCommand that follows the generated Trajectory
    RamseteCommand ramseteCommand = new RamseteCommand(
      //Pass in Trajectory
      trajectory,
      //Get position
      m_DriveTrain::getPose,
      //Create RamseteController, pass in B and Zeta values
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      //Create a FeedForward, pass in Position, Velocity, and Acceleration volts
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
      //Pass in DriveKinematics
      Constants.kDriveKinematics,
      //Pass in wheel speeds in METERS/SECOND
      m_DriveTrain::getWheelSpeeds,
      //Create two PID controllers (for wheels?)
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      //Allows RamseteCommand to access tankDriveVolts method, allows RamseteCommand to move wheels
      m_DriveTrain::tankDriveVolts,
      //Tells RamseteCommand the name of the DriveTrain we created
      m_DriveTrain);
    //Run RamseteCommand, then stop turning the wheels.
    return ramseteCommand.andThen(() -> m_DriveTrain.tankDriveVolts(0, 0));
  }
}
