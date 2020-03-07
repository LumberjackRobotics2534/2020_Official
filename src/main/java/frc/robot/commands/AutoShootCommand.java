package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutoShootCommand extends CommandBase {
  private Shooter m_shooter;
  Elevator m_Elevator;
  private double equRpm;
  private double actualRpm;
  private double distance = 0.0;
  public static boolean shooterReady = false;
  private boolean topBallPresence = false;
  private boolean wastopBallPresent = false;
  private static int shotsTaken = 0;
  private int targetShots;
  Timer shooterTimer = new Timer();
  
  public AutoShootCommand(Shooter _shooter, int _targetShots) {
    m_shooter = _shooter;
    targetShots = _targetShots;
    addRequirements(m_shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shotsTaken = 0;
    shooterTimer.reset();
    shooterTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getEquationVelocity();
    getShotsTaken();
    double targetVelocity_UnitsPer100ms =  -equRpm* 2048 / 600;
    m_shooter.shoot(targetVelocity_UnitsPer100ms);
    actualRpm = m_shooter.getAngularVelocity();
    if (actualRpm > equRpm - equRpm*Constants.acceptableRpmError 
    && actualRpm < equRpm + equRpm*Constants.acceptableRpmError
    && (shooterTimer.get() > 1.25)){
      shooterReady = true; 
    } else {
      shooterReady = false;
    }
    //.println("worked");
  }

  public int getShotsTaken(){
    topBallPresence = Elevator.topBallPresence();
    if(topBallPresence){
      wastopBallPresent = true;
    } else if(topBallPresence == false && wastopBallPresent){
      shotsTaken++;
      wastopBallPresent = false;
    }
    return shotsTaken;
  }

  public void getEquationVelocity() {
    distance = Turret.getDistance();
    if(distance >= 67 && distance <= 127){
     equRpm = (0.0000422)*Math.pow(distance, 4)*-1 + 0.01756*Math.pow(distance, 3) + -2.706*Math.pow(distance, 2) + 183.9*Math.pow(distance, 1) - 2174;
    } else if(distance > 127 && distance < 169){
     equRpm = ((Math.pow(distance - 127, 2))/12)+2547.728;
    } else if(distance >= 169 && distance <= 400){
      equRpm = (0.0000002801)*Math.pow(distance, 4) - 0.0002518*Math.pow(distance, 3) + 0.08358*Math.pow(distance, 2) - 12.15*Math.pow(distance, 1) + 5653;
    } else{
      equRpm = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    m_shooter.shooterOff();
    shooterReady = false;
    Turret.lightsEnabled(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shotsTaken == targetShots){
      return true;
    } else{
      return false;
    } 
  }
}
