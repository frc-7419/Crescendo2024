// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class ShootNote extends Command {
  private Shooter shooter;
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

  /** Creates a new ShootNotes. */
  public ShootNote(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = new Shooter();
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kS", kS);
    SmartDashboard.putNumber("kV", kV);
    SmartDashboard.putNumber("ShooterVelocity", velocitySetpoint);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = SmartDashboard.getNumber(getName(), kP);
    kI = SmartDashboard.getNumber(getName(), kI);
    kD = SmartDashboard.getNumber(getName(), kD);
    kS = SmartDashboard.getNumber(getName(), kS);
    kV = SmartDashboard.getNumber(getName(), kV);
    velocitySetpoint = SmartDashboard.getNumber("ShooterVelocity", velocitySetpoint);
    feedForwardTop = new SimpleMotorFeedforward(kS, kV);
    feedForwardBottom = new SimpleMotorFeedforward(kS, kV);
    shooterPIDTop = new PIDController(kP, kI, kD);
    shooterPIDBottom = new PIDController(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTopSpeed(feedForwardTop.calculate(velocitySetpoint) + shooterPIDTop.calculate(shooter.getTopVelocity(), velocitySetpoint));
    shooter.setBottomSpeed(feedForwardBottom.calculate(velocitySetpoint) + shooterPIDBottom.calculate(shooter.getBottomVelocity(), velocitySetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setBothSpeed(0);
    shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
