// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunShooterWithIntake extends Command {
  /** Creates a new RunShooter. */
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private double power;

  public RunShooterWithIntake(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double power) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.coast();
    shooterSubsystem.setTopSpeed(power);
    shooterSubsystem.setBottomSpeed(power);
    intakeSubsystem.setSerializerSpeed(power);
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
    intakeSubsystem.setSerializerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
