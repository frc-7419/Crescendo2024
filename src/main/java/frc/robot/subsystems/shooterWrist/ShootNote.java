// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunSerializer;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.RunShooterWithPID;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  public ShootNote(ShooterWrist shooterWrist, ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, IntakeSubsystem intakeSubsystem, double setpoint) {
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new WaitCommand(3.0),
            new RunShooter(shooterSubsystem, 0.7)
          ),
          new ParallelDeadlineGroup(
            new RunSerializer(intakeSubsystem),
            new RunShooter(shooterSubsystem, 0.7)
          )
          // new ParallelDeadlineGroup(
          //   new RunSerializer(intakeSubsystem),
          //   new RunShooterWithPID(shooterSubsystem, 3000, 3000)
          // )
      ),
        new RaiseShooterWithPID(shooterWrist, setpoint)
    )
    );
  }
}
