package frc.robot.subsystems.drive;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
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
    turnPID = new PIDController(0.08, 0, 0);
    turnPID.setTolerance(3);
    turnPID.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    Pose2d pose = drivetrain.getState().Pose;

    Translation2d translationalPose = pose.getTranslation();

    Translation2d vectorDiff = FieldConstants.SPEAKER_POSE.minus(translationalPose);
    
    double curAngle = (pose.getRotation().getDegrees());

    double targetAngle = (Math.atan(vectorDiff.getY()/vectorDiff.getX()) * (180/Math.PI));

    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", curAngle);
    // turnPID.enableContinuousInput(-180, 180);
    double output = -turnPID.calculate(curAngle, targetAngle);

    SmartDashboard.putNumber("Rot Output", output);

    drivetrain.setControl(driveRequest.withRotationalRate(-output));
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
