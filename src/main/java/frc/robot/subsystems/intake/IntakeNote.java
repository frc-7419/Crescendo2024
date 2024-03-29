// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeNote(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSubsystem.coast();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setSpeed(0.85);
        intakeSubsystem.setSerializerSpeed(0.3);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSerializerSpeed(0);
        intakeSubsystem.setSpeed(0);
        intakeSubsystem.brakeSerializer();
        intakeSubsystem.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return intakeSubsystem.frontBeamBreakIsTriggered();
    }
}
