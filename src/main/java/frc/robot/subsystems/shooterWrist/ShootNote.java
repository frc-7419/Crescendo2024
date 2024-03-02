// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.shooter.RunShooterWithPID;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  public ShootNote(ShooterWrist shooterWrist, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, double setpoint) {
    addCommands(
      // new ParallelCommandGroup(
      //   new RaiseShooterWithMotionMagic(shooterWrist, setpoint), 
      //   new RunShooterWithPID(shooterSubsystem, 3000, 3000)),
      // new ParallelCommandGroup(
      //   new RaiseShooterWithMotionMagic(shooterWrist, setpoint), 
      //   new RunShooterWithPID(shooterSubsystem, 3000, 3000),
      //   new RunIntake(intakeSubsystem, 0.5)
      //   ),
      // new RaiseShooterWithMotionMagic(shooterWrist, ArmConstants.armOffset + 2.0/360)
    );
  }
}
