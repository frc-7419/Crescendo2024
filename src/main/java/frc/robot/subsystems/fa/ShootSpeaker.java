// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShootSpeaker extends Command {
  private ShooterSubsystem shooterSubsystem;
  private SimpleMotorFeedforward feedForwardTop;
  private SimpleMotorFeedforward feedForwardBottom;
  private PIDController shooterPIDTop;
  private PIDController shooterPIDBottom;

  private double kP;
  private double kI;
  private double kD;
  private double kS;
  private double kV;
  private double velocitySetpoint;

  /** Creates a new ShootSpeaker. */
  public ShootSpeaker(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("kP",0);
    kI = SmartDashboard.getNumber("kI",0);
    kD = SmartDashboard.getNumber("kD",0);
    kS = SmartDashboard.getNumber("kS",0);
    kV = SmartDashboard.getNumber("kV",0);
    velocitySetpoint = SmartDashboard.getNumber("ShooterVelocity", 0);

    feedForwardTop = new SimpleMotorFeedforward(kS, kV);
    feedForwardBottom = new SimpleMotorFeedforward(kS, kV);
    shooterPIDTop = new PIDController(kP, kI, kD);
    shooterPIDBottom = new PIDController(kP, kI, kD);

    shooterSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ffTop = feedForwardTop.calculate(velocitySetpoint);
    double ffBottom = feedForwardBottom.calculate(velocitySetpoint);
    double PIDTop = shooterPIDTop.calculate(shooterSubsystem.getTopVelocity(), velocitySetpoint);
    double PIDBottom = shooterPIDBottom.calculate(shooterSubsystem.getBottomVelocity(), velocitySetpoint);
    shooterSubsystem.setTopSpeed(ffTop + PIDTop);
    shooterSubsystem.setBottomSpeed(ffBottom + PIDBottom);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterPIDTop.atSetpoint() && shooterPIDBottom.atSetpoint();
  }
}
