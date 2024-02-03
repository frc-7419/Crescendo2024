// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RunShooterWristToSetpoint extends Command {
  private ShooterWrist shooterWrist;
  private double setpoint;
  private ProfiledPIDController shooterWristPIDController;
  private double feedForward = (0.6/12)/2.67;
  private double feedForwardPower;

  /** Creates a new ShootNotes. */
  public RunShooterWristToSetpoint(ShooterWrist shooterWrist, double setpoint) {
    this.shooterWrist = shooterWrist;
    this.setpoint = setpoint;
    this.shooterWristPIDController 
      = new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(15, 15));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWristPIDController.setGoal(setpoint);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      feedForwardPower = feedForward * Math.cos(shooterWrist.getRadians());
      SmartDashboard.putNumber("Current Arm Setpoint", shooterWristPIDController.getGoal().position);
      double armPower = -shooterWristPIDController.calculate(shooterWrist.getPosition());
      armPower += Math.copySign(feedForwardPower, armPower);
      SmartDashboard.putNumber("armSetpointPower", armPower);
      shooterWrist.setPower(armPower);
  }

  public void visionExecute() {
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.setPower(0);
    shooterWrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterWristPIDController.atGoal();
  }
}
