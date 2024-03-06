// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.RunShooterWithIntake;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterWrist.RaiseShooterWithVision;
// import frc.robot.subsystems.shooterWrist.RunShooterWristToSetpoint;
import frc.robot.subsystems.shooterWrist.ShooterWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(ShooterSubsystem shooterSubsystem, ShooterWrist shooterWrist, IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain) {
    addCommands(
      
      // new RunShooter(shooterSubsystem, intakeSubsystem, 0.7).withTimeout(2).deadlineWith(new WaitCommand(1).andThen(new RunIntake(intakeSubsystem, 1).withTimeout(0.5)))
      

    );
  }
}
