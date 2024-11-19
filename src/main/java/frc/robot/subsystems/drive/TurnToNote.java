// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.wrapper.NoteVisionSubsystem;

public class TurnToNote extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PIDController turnPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
          .withDeadband(RobotConstants.kMaxSpeed * 0.05);
  private final NoteVisionSubsystem vision;

  /** Creates a new TurnToNote. */
  public TurnToNote(CommandSwerveDrivetrain drivetrain, NoteVisionSubsystem vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    turnPID = new PIDController(0, 0, 0); // arbitrary values
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setVisionMeasurementStdDevs(VisionConstants.VISION_STDS);
    turnPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = drivetrain.getState().Pose;

    Translation2d translationalPose = pose.getTranslation();

    Translation2d vectorDiff = vision.getNotePose().getTranslation().minus(translationalPose);

    double curAngle = (pose.getRotation().getDegrees());

    double targetAngle = (Math.atan(vectorDiff.getY() / vectorDiff.getX()) * (180 / Math.PI));

    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", curAngle);
    // turnPID.enableContinuousInput(-180, 180);
    double output = -turnPID.calculate(curAngle, targetAngle);

    SmartDashboard.putNumber("Rot Output", output);

    drivetrain.setControl(driveRequest
            .withVelocityX(output)
            .withRotationalRate(-output));
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
