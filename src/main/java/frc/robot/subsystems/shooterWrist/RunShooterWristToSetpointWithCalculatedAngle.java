// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunShooterWristToSetpointWithCalculatedAngle extends SequentialCommandGroup {
  private CommandXboxController joystick;
  private ShooterWrist shooterWrist;
  private CommandSwerveDrivetrain drivetrain;

  public RunShooterWristToSetpointWithCalculatedAngle(ShooterWrist shooterWrist, CommandSwerveDrivetrain drivetrain) {
    this.shooterWrist = shooterWrist;
    this.drivetrain = drivetrain;

    double angle = drivetrain.calculateAngle();
    SmartDashboard.putNumber("ShooterMAXCALC", angle);

    //TODO: add shooting later
    addCommands(
      new RunShooterWristToSetpoint(shooterWrist, angle)
    );
  }
}
