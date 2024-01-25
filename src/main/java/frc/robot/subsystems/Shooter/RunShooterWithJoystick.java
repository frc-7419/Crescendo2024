// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunShooterWithJoystick extends Command {
  private ShooterSubsystem shooterSubsystem;
  private CommandXboxController joystick;
  
  public RunShooterWithJoystick(ShooterSubsystem shooterSubsystem, CommandXboxController joystick) {
    this.shooterSubsystem = shooterSubsystem;
    this.joystick = joystick;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getLeftTriggerAxis() > 0.05){
      shooterSubsystem.setSerialSpeed(0.5);
    }
    else {
      shooterSubsystem.setSerialSpeed(0);
    }
    if(joystick.getRightTriggerAxis() > 0.05) {
      shooterSubsystem.setBothSpeed(1);
    }
    else {
      shooterSubsystem.setBothSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
