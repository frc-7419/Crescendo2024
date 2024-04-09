// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PulseRumble extends Command {
  private final XboxController controller;
  private long startTime;

  /** Creates a new PulseRumble. */
  public PulseRumble(XboxController controller) {
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.setRumble(XboxController.RumbleType.kLeftRumble, 1);
    controller.setRumble(XboxController.RumbleType.kRightRumble, 1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(XboxController.RumbleType.kLeftRumble, 0);
    controller.setRumble(XboxController.RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - startTime >= 1000;
  }
}
