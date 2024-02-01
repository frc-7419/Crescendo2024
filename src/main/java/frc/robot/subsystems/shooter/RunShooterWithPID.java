// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RunShooterWithPID extends Command {
  /** Creates a new RunShooterWithPID. */
  private ShooterSubsystem shooterSubsystem;
  private PIDController topShooterPidController = new PIDController(0.2, 0, 0);
  private PIDController bottomShooterPidController = new PIDController(0.2, 0, 0);

  public RunShooterWithPID(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topShooterPidController.reset();
    topShooterPidController.setSetpoint(ShooterConstants.maxVelocity);
    topShooterPidController.setTolerance(50);

    bottomShooterPidController.reset();
    bottomShooterPidController.setSetpoint(-ShooterConstants.maxVelocity);
    bottomShooterPidController.setTolerance(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTopSpeed(topShooterPidController.calculate(shooterSubsystem.getTopVelocity()));
    shooterSubsystem.setBottomSpeed(bottomShooterPidController.calculate(shooterSubsystem.getBottomVelocity()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (bottomShooterPidController.atSetpoint()  && topShooterPidController.atSetpoint());
  }
}
