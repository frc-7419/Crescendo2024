// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.shooter;

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.intake.IntakeSubsystem;
// import frc.robot.subsystems.intake.RunSerializer;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class RunShooterToAmp extends SequentialCommandGroup {
//   /** Creates a new RunShooterToAmp. */
//   public RunShooterToAmp(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
//     addCommands(
//       new ParallelDeadlineGroup(
//         new SequentialCommandGroup(
//           new RunShooterWithPID(shooterSubsystem,800*1.15, 1200*1.15).withTimeout(1.0),
//           new ParallelDeadlineGroup(
//             new RunSerializer(intakeSubsystem),
//             new RunShooterWithPID(shooterSubsystem,800*1.15, 1200*1.15)
//           )
//         ),
//         new RunCommand(new RaiseShooterWithPID(shooterWrist, (22.0)/360))
//       )
//     );
//   }
// }
