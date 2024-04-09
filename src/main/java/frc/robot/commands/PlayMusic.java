// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class PlayMusic extends Command {
  // private final Orchestra orchestra;
  // private final TalonFX[] motors;

  /** Creates a new PlayMusic. */
  public PlayMusic() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // orchestra = new Orchestra();
  // motors = { new TalonFX(1, "Ryan Biggee"), new TalonFX(2, "Ryan Biggee"), new TalonFX(3, "Ryan Biggee"), new TalonFX(4, "Ryan Biggee"), new TalonFX(5, "Ryan Biggee"), new TalonFX(6, "Ryan Biggee"), new TalonFX(7, "Ryan Biggee"), new TalonFX(8, "Ryan Biggee")};
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
