// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.wrapper.VisionWrapper;

public class TurnToAmp extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(RobotConstants.kMaxSpeed * 0.05);
  private VisionWrapper vision;

  /** Creates a new TurnToSpeaker. */
  public TurnToAmp(CommandSwerveDrivetrain drivetrain, VisionWrapper vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setVisionMeasurementStdDevs(VisionConstants.VISION_STDS);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EstimatedRobotPose[] estimates = vision.updatePoseEstimate();
    for (EstimatedRobotPose estimate : estimates) {
      if (estimate != null) {
        double angle = vision.headingToTag(9);
        System.out.println(angle + "hello");
        // drivetrain.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
        drivetrain.setControl(driveRequest.withRotationalRate(-angle*0.1));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
