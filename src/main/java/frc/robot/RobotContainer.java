package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.JoyDriveCommand;
import frc.robot.commands.ShootCommand;

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
  public static final DriveTrain m_DriveTrain = new DriveTrain();
  public static final ColorWheel m_ColorWheel = new ColorWheel();
  public static final Shooter m_Shooter = new Shooter();
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    buttonA.whenHeld(new ShootCommand(m_Shooter, buttonA));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
