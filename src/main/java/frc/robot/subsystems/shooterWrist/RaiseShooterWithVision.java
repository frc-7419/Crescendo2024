package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class RaiseShooterWithVision extends Command {
  private final ShooterWrist shooterWrist;
  private final CommandSwerveDrivetrain drivetrain;
  private final ProfiledPIDController shooterWristPIDController;
  private final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private final double feedForward = (2.1/12)/2.67;
  private double feedForwardPower;

  /** Creates a new ShootNotes. */
  public RaiseShooterWithVision(CommandSwerveDrivetrain drivetrain, ShooterWrist shooterWrist) {
    this.drivetrain = drivetrain;
    this.shooterWrist = shooterWrist;
    this.shooterWristPIDController 
      = new ProfiledPIDController(1.9, 0.07, 0.05, new TrapezoidProfile.Constraints(10, 0.1125));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    interpolatingDoubleTreeMap.put(1.25, 46.06/360);
    interpolatingDoubleTreeMap.put(1.51, 38.1/360);
    interpolatingDoubleTreeMap.put(1.73, 39.1/360);
    interpolatingDoubleTreeMap.put(2.02, 36.2/360);
    interpolatingDoubleTreeMap.put(2.3, 32.55/360);
    interpolatingDoubleTreeMap.put(2.58, 29.75/360);
    interpolatingDoubleTreeMap.put(2.89, 27.44/360);
    interpolatingDoubleTreeMap.put(3.23, 26.69/360);
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d pose = drivetrain.getState().Pose.getTranslation();

    Translation2d speakerPose = drivetrain.getSpeakerPose();

    double distance = pose.getDistance(speakerPose);
    SmartDashboard.putNumber("Distance to Speaker", distance);
    double setpoint = interpolatingDoubleTreeMap.get(distance);
     SmartDashboard.putNumber("Shooter Auto Angle", setpoint);

    shooterWristPIDController.setGoal(setpoint );
    shooterWrist.setPIDsetpoint(setpoint);
    // shooterWristPIDController.setGoal(7);
      feedForwardPower = feedForward * Math.cos(shooterWrist.getPositionInRadians());
      SmartDashboard.putNumber("Current Arm Setpoint", shooterWristPIDController.getGoal().position);
      double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
      armPower += Math.copySign(feedForwardPower, armPower);
      SmartDashboard.putNumber("armSetpointPower", armPower);
      shooterWrist.setPower(armPower * 12);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterWrist.setPower(0);
    shooterWrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterWristPIDController.atGoal();
  }
}
