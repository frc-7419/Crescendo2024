// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RunIntakeWithJoystick extends Command {
  private final Intake intakeSubsystem;
  private final CommandXboxController joystick;
  public RunIntakeWithJoystick(Intake intakeSubsystem, CommandXboxController joystick) {
    this.intakeSubsystem = intakeSubsystem;
    this.joystick = joystick;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.coast();
    intakeSubsystem.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(joystick.getRightTriggerAxis()) > 0.05) intakeSubsystem.setSpeed(joystick.getRightTriggerAxis());
    else if(Math.abs(joystick.getLeftTriggerAxis()) > 0.05) intakeSubsystem.setSpeed(-joystick.getLeftTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setVoltage(0);
    intakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
