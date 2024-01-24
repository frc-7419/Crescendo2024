// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShootNoteAmp extends Command {
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
  public ShootNoteAmp(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = new ShooterSubsystem();
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kS", kS);
    SmartDashboard.putNumber("kV", kV);
    velocitySetpointTop = 1.0;
    velocitySetpointBottom = 3.0; //tentative, replace with actual velocities
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
    feedForwardTop = new SimpleMotorFeedforward(kS, kV);
    feedForwardBottom = new SimpleMotorFeedforward(kS, kV);
    shooterPIDTop = new PIDController(kP, kI, kD);
    shooterPIDBottom = new PIDController(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTopSpeed(feedForwardTop.calculate(velocitySetpointTop) + shooterPIDTop.calculate(shooterSubsystem.getTopVelocity(), velocitySetpointTop));
    shooterSubsystem.setBottomSpeed(feedForwardBottom.calculate(velocitySetpointBottom) + shooterPIDBottom.calculate(shooterSubsystem.getBottomVelocity(), velocitySetpointBottom));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setBothSpeed(0);
    shooterSubsystem.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
