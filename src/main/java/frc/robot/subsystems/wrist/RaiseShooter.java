package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.RobotConstants.Action;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class RaiseShooter extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Wrist wrist;
  private ProfiledPIDController controller;
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 2.7/2.67, 0);
  private InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private Action state;

  public RaiseShooter(CommandSwerveDrivetrain drivetrain, Wrist wrist, Action state) {
    this.wrist = wrist;
    this.controller 
      = new ProfiledPIDController(Rotations.of(1.9).in(Degrees), Rotations.of(0.07).in(Degrees), Rotations.of(0.05).in(Degrees), new TrapezoidProfile.Constraints(Rotations.of(10).in(Degrees),Rotations.of(0.1125).in(Degrees)));

    interpolatingDoubleTreeMap.put(1.25, 46.06/360);
    interpolatingDoubleTreeMap.put(1.51, 38.1/360);
    interpolatingDoubleTreeMap.put(1.73, 39.1/360);
    interpolatingDoubleTreeMap.put(2.02, 36.2/360);
    interpolatingDoubleTreeMap.put(2.3, 32.55/360);
    interpolatingDoubleTreeMap.put(2.58, 29.75/360);
    interpolatingDoubleTreeMap.put(2.89, 27.44/360);
    interpolatingDoubleTreeMap.put(3.23, 26.69/360);

    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Measure<Angle> setpoint = Degrees.of(0);
    wrist.coast();
    wrist.setPower(0);
    controller.setTolerance(ShooterConstants.SetpointThreshold);
   
    switch (state) {
      case STOW:
        setpoint = Degrees.of(0.0);
      case AMP:
        setpoint = Degrees.of(60.0);
      case SPEAKER:
        setpoint = Rotations.of(interpolatingDoubleTreeMap.get(drivetrain.getDistance()));
    }

    controller.setGoal(setpoint.in(Degrees));
    SmartDashboard.putNumber("Arm Setpoint", setpoint.in(Degrees));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double feedForward = armFeedforward.calculate(wrist.getPositionInRadians(), wrist.getVelocityInRadians());
      double armPower = controller.calculate(wrist.getPositionInDegrees());
      armPower += Math.copySign(feedForward, armPower);
      
      wrist.setPower(armPower);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPower(0);
    wrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
