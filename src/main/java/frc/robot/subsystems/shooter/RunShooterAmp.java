// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterAmp extends Command {
  /** Creates a new RunShooter. */
  private ShooterSubsystem shooterSubsystem;
  private double topPower;
  private double bottomPower;

  public RunShooterAmp(ShooterSubsystem shooterSubsystem, double topPower, double bottomPower) {
    this.shooterSubsystem = shooterSubsystem;
    this.topPower = topPower;
    this.bottomPower = bottomPower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.coast();
    shooterSubsystem.setTopSpeed(topPower);
    shooterSubsystem.setBottomSpeed(bottomPower);
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
