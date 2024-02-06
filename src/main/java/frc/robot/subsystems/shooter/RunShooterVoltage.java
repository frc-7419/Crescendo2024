package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterVoltage extends Command {
  /** Creates a new RunShooter. */
  private ShooterSubsystem shooterSubsystem;
  private double voltage;

  public RunShooterVoltage(ShooterSubsystem shooterSubsystem, double voltage) {
    this.shooterSubsystem = shooterSubsystem;
    this.voltage = voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.coast();
    shooterSubsystem.setBothVoltage(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
    shooterSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
