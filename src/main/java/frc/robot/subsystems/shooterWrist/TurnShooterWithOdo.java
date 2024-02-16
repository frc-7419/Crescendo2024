package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TurnShooterWithOdo extends Command {
  private ShooterWrist shooterWrist;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private ProfiledPIDController pidController = new ProfiledPIDController(1.5, 0, 0.05,
      new TrapezoidProfile.Constraints(10, 0.1125));

  private double feedForward = (0.6 / 12) / 2.67;

  public TurnShooterWithOdo(CommandSwerveDrivetrain commandSwerveDrivetrain, ShooterWrist shooterWrist) {
    this.shooterWrist = shooterWrist;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    addRequirements(shooterWrist);
  }

  @Override
  public void initialize() {
    interpolatingDoubleTreeMap.put(5.68, 10.2);
    interpolatingDoubleTreeMap.put(5.5, 14.0);
    interpolatingDoubleTreeMap.put(6.02,3.0);
  }

  @Override
  public void execute() {
    Pose2d pose = commandSwerveDrivetrain.getState().Pose;
    double distX = pose.getX() - 0;
    double distY = pose.getY() - 5;
    double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));

    double setpoint = interpolatingDoubleTreeMap.get(dist);

    double feedForwardPower = feedForward * Math.cos(shooterWrist.getRadians());
    pidController.setGoal(setpoint);
    double armPower = pidController.calculate(shooterWrist.getPosition()*360);
    armPower += Math.copySign(feedForwardPower, armPower);
    SmartDashboard.putNumber("armSetpointPower", armPower);
    shooterWrist.setPower(armPower);
  }

  @Override
  public void end(boolean interrupted) {
    shooterWrist.setPower(0);
    shooterWrist.brake();
  }

  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }
}
