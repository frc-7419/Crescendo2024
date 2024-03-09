package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.ShooterConstants;

public class RaiseShooterWithPID extends Command {
  private final ShooterWrist shooterWrist;
  private final ProfiledPIDController shooterWristPIDController;
  private final double feedForward = (0.9/12)/2.67;
  private double feedForwardPower;
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.02809*2.5, 0.01*1.5);
  private double setpoint;
  private double setpointInDegrees;

  /** Creates a new ShootNotes. */
  public RaiseShooterWithPID(ShooterWrist shooterWrist, double setpoint) {
    this.shooterWrist = shooterWrist;
    this.setpoint = setpoint;
    this.shooterWristPIDController = new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(20,1.5));
    // this.shooterWristPIDController 
    //   = new ProfiledPIDController(Rotations.of(1.9).in(Degrees), Rotations.of(0.07).in(Degrees), Rotations.of(0.05).in(Degrees), new TrapezoidProfile.Constraints(Rotations.of(10).in(Degrees),Rotations.of(0.1125).in(Degrees)));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWrist.setPIDsetpoint(setpoint);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
    shooterWristPIDController.setGoal(setpoint);
    shooterWristPIDController.reset(shooterWrist.getPosition());
    SmartDashboard.putNumber("Arm Setpoint", setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
      double armError = setpoint - shooterWrist.getPosition();
      armPower = armPower + armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocityInRadians());
      shooterWrist.setPower(armPower*12);

      SmartDashboard.putNumber("Arm Error", armError);
      SmartDashboard.putNumber("Arm power with ff", armPower);
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
    //return shooterWristPIDController.atGoal();
  }
}
