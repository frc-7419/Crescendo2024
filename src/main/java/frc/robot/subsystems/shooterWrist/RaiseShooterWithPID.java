package frc.robot.subsystems.shooterWrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class RaiseShooterWithPID extends Command {
  private ShooterWrist shooterWrist;
  private ProfiledPIDController shooterWristPIDController;
  private double feedForward = (2.1/12)/2.67;
  private double feedForwardPower;
  private double setpoint;

  /** Creates a new ShootNotes. */
  public RaiseShooterWithPID(ShooterWrist shooterWrist, double setpoint) {
    this.shooterWrist = shooterWrist;
    this.setpoint = setpoint;
    this.shooterWristPIDController 
      = new ProfiledPIDController(1.9, 0.07, 0.05, new TrapezoidProfile.Constraints(10, 0.1125));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterWristPIDController.setGoal(setpoint);
    // shooterWristPIDController.setGoal(7);
      feedForwardPower = feedForward*0.8;
      //* Math.cos(shooterWrist.getRadians()-(52 * Math.PI/180));
      double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition().in(Degrees));
      armPower += Math.copySign(feedForwardPower, armPower);
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
