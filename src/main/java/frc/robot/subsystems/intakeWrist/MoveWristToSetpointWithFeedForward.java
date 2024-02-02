// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeWrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.IntakeWristConstants;;

public class MoveWristToSetpointWithFeedForward extends Command {
  /** Creates a new MoveWristToSetpointWithFeedForward. */
  private ArmFeedforward feedforward;
  private ProfiledPIDController wristPIDController;

  private double setpoint;

  private IntakeWristSubsystem wrist;

  public MoveWristToSetpointWithFeedForward(IntakeWristSubsystem wrist, double setpoint) {
    this.feedforward = IntakeWristConstants.wristFeedForward;
    this.wristPIDController = new ProfiledPIDController(IntakeWristConstants.kP, IntakeWristConstants.kI,
        IntakeWristConstants.kD, IntakeWristConstants.constraints);
    this.setpoint = setpoint;
    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.coast();
    wrist.setVoltage(0);
  }

  @Override
  public void execute() {
    while (Math.abs(setpoint - wrist.getPosition()) > IntakeWristConstants.SetpointThreshold) {
      wristPIDController.setGoal(setpoint);
      wrist.setVoltage(wristPIDController.calculate(wrist.getPosition())
          + feedforward.calculate(setpoint, wrist.getVelocity()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    wrist.brake();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
