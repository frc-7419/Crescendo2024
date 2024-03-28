// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunSerializer;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteFar extends SequentialCommandGroup {
    public ShootNoteFar(ShooterWrist shooterWrist, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, IntakeSubsystem intakeSubsystem, double setpoint) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                        new RaiseShooterWithPID(shooterWrist, setpoint),
                                        new WaitCommand(0.5),
                                        new RunShooter(shooterSubsystem, 0.6)
                                ),
                                new ParallelDeadlineGroup(
                                        new RunSerializer(intakeSubsystem).withTimeout(0.5),
                                        new RunShooter(shooterSubsystem, 0.6)
                                )
                        )
                       
                )
        );
    }
}
