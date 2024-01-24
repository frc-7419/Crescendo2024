// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

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
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    kP = 0.1; 
    kI = 0;
    kD = 0;
    kS = 0;
    kV = 0;
    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kI", kI);
    // SmartDashboard.putNumber("kD", kD);
    // SmartDashboard.putNumber("kS", kS);
    // SmartDashboard.putNumber("kV", kV);
    // SmartDashboard.putNumber("ShooterVelocity", velocitySetpoint);
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
    velocitySetpoint = SmartDashboard.getNumber("ShooterVelocity", velocitySetpoint);
    feedForwardTop = new SimpleMotorFeedforward(kS, kV);
    feedForwardBottom = new SimpleMotorFeedforward(kS, kV);
    shooterPIDTop = new PIDController(kP, kI, kD);
    shooterPIDBottom = new PIDController(kP, kI, kD);

    shooterSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTopSpeed(feedForwardTop.calculate(velocitySetpoint) + shooterPIDTop.calculate(shooterSubsystem.getTopVelocity(), velocitySetpoint));
    shooterSubsystem.setBottomSpeed(feedForwardBottom.calculate(velocitySetpoint) + shooterPIDBottom.calculate(shooterSubsystem.getBottomVelocity(), velocitySetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
