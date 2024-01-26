// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunShooterWristWithJoystick extends Command {
  /** Creates a new RunArmWithJoystick. */
  private CommandXboxController joystick;
  private ShooterWrist shooterWrist;
  private ArmFeedforward armFeedforward;
  private double maxPower = 0.1;
  public RunShooterWristWithJoystick(ShooterWrist shooterWrist, CommandXboxController joystick) {
    this.shooterWrist = shooterWrist;
    this.armFeedforward = new ArmFeedforward(0, 0.1, 0);
    this.joystick = joystick;
    addRequirements(shooterWrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double feedForward = armFeedforward.calculate(, maxPower);
    double feedForward = 0.5/12/2.67;
    if (Math.abs(joystick.getLeftY())>0.05){
      shooterWrist.coast();
      double armPower = maxPower * joystick.getLeftY();
      shooterWrist.setPower(armPower+feedForward);
    } else {
      shooterWrist.setPower(feedForward);
      shooterWrist.brake();

    }
    // shooterWrist.setPower(joystick.getLeftY());
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
    return false;
  }
}