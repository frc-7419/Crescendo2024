package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;

public class TurnToSpeaker extends Command {
  private PIDController turnPID;
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

  public TurnToSpeaker(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setVisionMeasurementStdDevs(VisionConstants.VISION_STDS);

    turnPID = new PIDController(0.1, 0, 0);
    turnPID.setTolerance(3);
  }

  @Override
  public void execute() {
    Pose2d pose = drivetrain.getState().Pose;
    double distX = pose.getX() - 0;
    double distY = pose.getY() - 5;
    double curAngle = (360+pose.getRotation().getDegrees()) % 360;

    double targetAngle = Math.atan(distY/distX) * (180/Math.PI) + 180;

    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", curAngle);

    double output = turnPID.calculate(curAngle, targetAngle);

    SmartDashboard.putNumber("Rot Output", output);

    drivetrain.setControl(driveRequest.withRotationalRate(output));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
  }

  @Override
  public boolean isFinished() {
    return turnPID.atSetpoint();
  }
}
