// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.wrapper.VisionWrapper;

public class DrivetrainWithVision extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive;
  private final XboxController xboxController;
  private final VisionWrapper vision;
  private Double drivetrainPower;

  /** Creates a new DrivetrainWithVision. */
  public DrivetrainWithVision(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive,
      XboxController xboxController, VisionWrapper vision) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.xboxController = xboxController;
    this.vision = vision;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setVisionMeasurementStdDevs(VisionConstants.VISION_STDS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // i think theres input delay
    drivetrainPower = vision.calculateRotation(7);
    System.out.println(drivetrainPower);
    drivetrain.setControl(drive.withVelocityX(-xboxController.getLeftY() * RobotConstants.kMaxSpeed) // Drive forward
        // with // negative
        // Y (forward)
        .withVelocityY(-xboxController.getLeftX() * RobotConstants.kMaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(
            (xboxController.getXButton()) ? (drivetrainPower)
                : (-xboxController.getRightX() * RobotConstants.kMaxAngularRate)) // Drive
    // counterclockwise
    // with
    // negative X (left)
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
