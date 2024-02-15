// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.shooterWrist.RunShooterWristToSetpoint;
import frc.robot.subsystems.shooterWrist.ShooterWrist;
import frc.robot.wrapper.VisionWrapper;

public class TurnShooterToTag extends Command {

    private ShooterWrist shooter;
    // private final SwerveRequest.FieldCentric driveRequest = new
    // SwerveRequest.FieldCentric()
    // .withDeadband(RobotConstants.kMaxSpeed * 0.05);
    private VisionWrapper vision;
    private ShooterWrist shooterWrist;
    private ProfiledPIDController shooterWristPIDController;
    private double feedForward = (0.6 / 12) / 2.67;
    private double feedForwardPower;

    /** Creates a new TurnToSpeaker. */
    public TurnShooterToTag(ShooterWrist shooter, VisionWrapper vision) {
        this.shooter = shooter;
        this.vision = vision;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
        this.shooterWrist = shooterWrist;
        this.shooterWristPIDController 
            = new ProfiledPIDController(1.5, 0, 0.05, new TrapezoidProfile.Constraints(10, 0.1125));
        addRequirements(shooterWrist);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // shooter.setVisionMeasurementStdDevs(VisionConstants.VISION_STDS);
        shooterWrist.coast();
        shooterWrist.setPower(0);
        shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
        double setpoint = Math.atan(80/vision.distanceToTag(9));
        shooterWristPIDController.setGoal(setpoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        EstimatedRobotPose[] estimates = vision.updatePoseEstimate();
        for (EstimatedRobotPose estimate : estimates) {
            if (estimate != null) {
                // double angle = vision.headingToTag(9);
                // System.out.println(angle + "hello");
                // drivetrain.addVisionMeasurement(estimate.estimatedPose.toPose2d(),
                // estimate.timestampSeconds);
                // shooter.setControl(driveRequest.withRotationalRate(-angle*0.1));
                feedForwardPower = feedForward * Math.cos(shooterWrist.getRadians());
                SmartDashboard.putNumber("Current Arm Setpoint", shooterWristPIDController.getGoal().position);
                double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
                armPower += Math.copySign(feedForwardPower, armPower);
                SmartDashboard.putNumber("armSetpointPower", armPower);
                shooterWrist.setPower(armPower);
            }
        }
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
        shooterWrist.setPower(0);
        shooterWrist.brake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooterWristPIDController.atGoal();
    }
}
