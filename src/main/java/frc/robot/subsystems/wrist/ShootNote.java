// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.RobotConstants.Action;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunSerializer;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  public ShootNote(Wrist wrist, Shooter shooter, CommandSwerveDrivetrain drivetrain, Intake intake) {
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new WaitCommand(2.5),
            new RunShooter(shooter, 0.6)
          ),
          new ParallelDeadlineGroup(
            new RunSerializer(intake).withTimeout(0.5),
            new RunShooter(shooter, 0.6)
          )
      ),
        new RaiseShooter(drivetrain, wrist, Action.SPEAKER)
    )
    );
  }
}
