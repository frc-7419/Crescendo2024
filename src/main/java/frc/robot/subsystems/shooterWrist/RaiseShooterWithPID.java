package frc.robot.subsystems.shooterWrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
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
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 2.7/2.67, 0);
  private double setpoint;



  /** Creates a new ShootNotes. */
  public RaiseShooterWithPID(ShooterWrist shooterWrist, double setpoint) {
    this.shooterWrist = shooterWrist;
    this.setpoint = setpoint;
    // this.shooterWristPIDController 
    //   = new ProfiledPIDController(Rotations.of(1.9).in(Degrees), Rotations.of(0.07).in(Degrees), Rotations.of(0.05).in(Degrees), new TrapezoidProfile.Constraints(Rotations.of(10).in(Degrees),Rotations.of(0.1125).in(Degrees)));
    this.shooterWristPIDController 
      = new ProfiledPIDController(0.1,0,0, new TrapezoidProfile.Constraints(0.01,0.001));
    addRequirements(shooterWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterWrist.coast();
    shooterWrist.setPower(0);
    shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
    // shooterWristPIDController.setGoal(Degrees.of(setpoint).in(Degrees));
    shooterWristPIDController.setGoal(setpoint * 360);
    SmartDashboard.putNumber("Arm Setpoint", setpoint * 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //feedForwardPower = armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocity());
      double armPower = -shooterWristPIDController.calculate(shooterWrist.getPositionInDegrees());
      SmartDashboard.putNumber("Arm Error", setpoint*360 - shooterWrist.getPositionInDegrees());
      SmartDashboard.putNumber("armie power", armPower);
      armPower += Math.copySign(feedForward, armPower);
      SmartDashboard.putNumber("arm power with ff", armPower);
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
