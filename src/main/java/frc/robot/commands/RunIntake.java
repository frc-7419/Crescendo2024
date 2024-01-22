// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  /** Creates a new RunIntake. */
  private Intake intake;
  private CommandXboxController joystick;
  public RunIntake(Intake intake, CommandXboxController joystick) {
    this.intake = intake;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(joystick.getRightTriggerAxis() > 0.05) {
      intake.coastActuator();
      intake.coastRun();
      intake.setActuatorPower(0.5);
      intake.setRunPower(0.5);
    }
    if(joystick.getLeftTriggerAxis() > 0.05) {
       intake.coastActuator();
      intake.coastRun();
      intake.setActuatorPower(0.5);
      intake.setRunPower(0.5);
    }
    else {
      intake.brakeActuator();
      intake.brakeRun();
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
 