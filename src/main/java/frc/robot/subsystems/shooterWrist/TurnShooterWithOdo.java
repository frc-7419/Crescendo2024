package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TurnShooterWithOdo extends Command {
  private ShooterWrist shooterWrist;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private ProfiledPIDController shooterWristPIDController;
  private InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private double feedForward = (2.1/12)/2.67;
  private double feedForwardPower;

  /** Creates a new ShootNotes. */
  public TurnShooterWithOdo(CommandSwerveDrivetrain commandSwerveDrivetrain, ShooterWrist shooterWrist) {
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.shooterWrist = shooterWrist;
    this.shooterWristPIDController 
      = new ProfiledPIDController(1.9, 0.07, 0.05, new TrapezoidProfile.Constraints(10, 0.1125));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    interpolatingDoubleTreeMap.put(2.06, 65.2/360);
    interpolatingDoubleTreeMap.put(2.8, 50.2/360);
    interpolatingDoubleTreeMap.put(3.4, 46.0/360);
    interpolatingDoubleTreeMap.put(3.8, 41.0/360);
    interpolatingDoubleTreeMap.put(4.0, 37.0/360);
    interpolatingDoubleTreeMap.put(4.5, 32.0/360);
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = commandSwerveDrivetrain.getState().Pose;
    double distX = pose.getX() - 0;
    double distY = pose.getY() - 5;
    double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
    double setpoint = interpolatingDoubleTreeMap.get(dist);
    shooterWristPIDController.setGoal(setpoint);
      feedForwardPower = feedForward * Math.cos(shooterWrist.getRadians());
      SmartDashboard.putNumber("Current Arm Setpoint", shooterWristPIDController.getGoal().position);
      double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
      armPower += Math.copySign(feedForwardPower, armPower);
      SmartDashboard.putNumber("armSetpointPower", armPower);
      shooterWrist.setPower(armPower);
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
