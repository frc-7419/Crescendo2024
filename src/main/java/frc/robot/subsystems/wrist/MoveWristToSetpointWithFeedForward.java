// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.WristConstants;

public class MoveWristToSetpointWithFeedForward extends Command {
  /** Creates a new MoveWristToSetpointWithFeedForward. */
  private final ArmFeedforward feedforward;
  private TrapezoidProfile wristMotorTrapezoidProfile;
  private final ProfiledPIDController wristPIDController;

  private double setpoint;

  private Wrist wrist;

  public MoveWristToSetpointWithFeedForward(Wrist wrist, double setpoint) {
    this.feedforward = WristConstants.wristFeedForward;
    this.wristPIDController =
      new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, wrist.getConstraints()); 
      // TODO: add kDT here
    this.setpoint = setpoint;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.coast();
    wrist.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Math.abs(setpoint - wrist.getPosition()) > WristConstants.SetpointThreshold){
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
