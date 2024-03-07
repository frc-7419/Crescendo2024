// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
  private Intake intake;
  public IntakeNote(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.coast();
    // intake.invertMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeed(0.7);
    intake.setSerializerSpeed(0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSerializerSpeed(0);
    intake.setSpeed(0);
    intake.brakeSerializer();
    intake.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.frontBeamBreakIsTriggered();
  }
}
