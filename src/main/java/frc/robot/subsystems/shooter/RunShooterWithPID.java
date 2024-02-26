// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterWithPID extends Command {
  /** Creates a new RunShooterWithPID. */
  private ShooterSubsystem shooterSubsystem;

  private double topV, bottomV;

  public RunShooterWithPID(ShooterSubsystem shooterSubsystem, double topVelocity, double bottomVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    topV = topVelocity;
    bottomV = bottomVelocity;
    SmartDashboard.putNumber("Top Setpoint", topV);
    SmartDashboard.putNumber("Bottom Setpoint", bottomV);


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topV = SmartDashboard.getNumber("Top Setpoint", topV);
    topV = SmartDashboard.getNumber("Top Setpoint", topV);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   shooterSubsystem.setRPM(topV, bottomV);

    SmartDashboard.putNumber("Bottom Setpoint", bottomV);
    SmartDashboard.putNumber("Bottom Velocity", shooterSubsystem.getBottomVelocity());
    SmartDashboard.putNumber("Top Setpoint", topV);
    SmartDashboard.putNumber("Top Velocity", shooterSubsystem.getTopVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0.0);
    shooterSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
