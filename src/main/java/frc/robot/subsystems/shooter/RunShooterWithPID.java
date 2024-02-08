// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RunShooterWithPID extends Command {
  /** Creates a new RunShooterWithPID. */
  private ShooterSubsystem shooterSubsystem;
  private PIDController topShooterPidController = new PIDController(0.001852, 0, 0);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806,0.015777);
  private PIDController bottomShooterPidController = new PIDController(0.001852, 0, 0);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.10894, 0.10806,0.015777);

  private double topV, bottomV;

  public RunShooterWithPID(ShooterSubsystem shooterSubsystem, double topVelocity, double bottomVelocity) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    topV = topVelocity;
    bottomV = bottomVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    topShooterPidController.reset();
    topShooterPidController.setSetpoint(topV);
    topShooterPidController.setTolerance(50);

    bottomShooterPidController.reset();
    bottomShooterPidController.setSetpoint(bottomV);
    bottomShooterPidController.setTolerance(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topPid = topShooterPidController.calculate(shooterSubsystem.getTopVelocity());
    double bottomPid = bottomShooterPidController.calculate(shooterSubsystem.getBottomVelocity());
    double topFor = topFeedforward.calculate(topV);
    double bottomFor = bottomFeedforward.calculate(bottomV);
    shooterSubsystem.setTopSpeed(topPid+topFor);
    shooterSubsystem.setBottomSpeed(bottomPid+bottomFor);
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
    return (bottomShooterPidController.atSetpoint()  && topShooterPidController.atSetpoint());
  }
}
