// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;

public class RunShooterToSetpoint extends Command {
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
  private double velocitySetpointTop;
  private double velocitySetpointBottom;

  /** Creates a new ShootNotes. */
  public RunShooterToSetpoint(ShooterSubsystem shooterSubsystem, double topSpeed, double bottomSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.velocitySetpointTop = topSpeed;
    this.velocitySetpointBottom = bottomSpeed;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber("kP", 0.1);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    kS = SmartDashboard.getNumber("kS", 2);
    kV = SmartDashboard.getNumber("kV", 0);
    // velocitySetpointTop = SmartDashboard.getNumber("ShooterVelocity", 0);
    // velocitySetpointBottom = SmartDashboard.getNumber("ShooterVelocity", 0);
    // velocitySetpointTop = 
    feedForwardTop = new SimpleMotorFeedforward(2, 0.1);
    feedForwardBottom = new SimpleMotorFeedforward(2, 0.1);
    shooterPIDTop = new PIDController(0.01, kI, kD);
    shooterPIDBottom = new PIDController(0.01, kI, kD);
    shooterPIDTop.setTolerance(ShooterConstants.tolerance);
    shooterPIDBottom.setTolerance(ShooterConstants.tolerance);
    shooterSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ffTop = feedForwardTop.calculate(velocitySetpointTop);
    double ffBottom = feedForwardBottom.calculate(velocitySetpointBottom);
    double PIDTop = shooterPIDTop.calculate(shooterSubsystem.getTopVelocity(), velocitySetpointTop);
    double PIDBottom = shooterPIDBottom.calculate(shooterSubsystem.getBottomVelocity(), velocitySetpointBottom);
    shooterSubsystem.setTopSpeed(ffTop + PIDTop);
    shooterSubsystem.setBottomSpeed(-1 *( ffBottom + PIDBottom));
    // shooterSubsystem.setTopSpeed(0.2);
    // shooterSubsystem.setBottomSpeed(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return shooterPIDTop.atSetpoint() && shooterPIDBottom.atSetpoint();
    return false;
  }
}
