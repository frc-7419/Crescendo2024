// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveShooterWristToSetpoint extends Command {
  /** Creates a new MoveShooterWristToSetpoint. */
  private ShooterWrist shooterWrist;
  private PIDController shooterWristPIDController;
  private double setpointRotations;

  public MoveShooterWristToSetpoint(ShooterWrist shooterWrist, double degree) {
    this.shooterWrist = shooterWrist;
    this.shooterWristPIDController = new PIDController(0.0000001, 0, 0);
    this.setpointRotations = shooterWrist.degreesToRotation(degree);
    addRequirements(shooterWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWristPIDController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double wristPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
    shooterWrist.setPower(Math.max(wristPower, 0.1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.brake();
    shooterWrist.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
