// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double power;

    /**
     * Creates a new RunIntake.
     */
    public RunIntake(IntakeSubsystem intakeSubsystem, double power) {
        this.intakeSubsystem = intakeSubsystem;
        this.power = power;
        addRequirements(intakeSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSubsystem.coast();
        intakeSubsystem.setSpeed(0);
        // intakeSubsystem.invertMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSubsystem.setSpeed(power);
        intakeSubsystem.setSerializerSpeed(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
        intakeSubsystem.setSerializerSpeed(0);
        intakeSubsystem.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}