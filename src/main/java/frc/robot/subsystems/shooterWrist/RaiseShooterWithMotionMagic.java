package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class RaiseShooterWithMotionMagic extends Command {
  private ShooterWrist shooterWrist;
  private ProfiledPIDController shooterWristPIDController;
  private double feedForward = (2.1/12)/2.67;
  private double feedForwardPower;
  private double setpoint;

  /** Creates a new ShootNotes. */
  public RaiseShooterWithMotionMagic(ShooterWrist shooterWrist, double setpoint) {
    this.shooterWrist = shooterWrist;
    this.setpoint = setpoint;
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.applyRequest(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterWrist.applyRequest(48/360);
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
    return false;
  }
}
