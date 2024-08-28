package frc.robot.subsystems.shooterWrist;

import edu.wpi.first.math.controller.ArmFeedforward;
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
    private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0.02809 * 2.5, 0.01 * 1.5);
    private final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
    private double setpoint;

    /**
     * Creates a new ShootNotes.
     */
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
    
            interpolatingDoubleTreeMap.put(1.128, 60.0 / 360);
            interpolatingDoubleTreeMap.put(1.97, 0.12);
            interpolatingDoubleTreeMap.put(2.29, 0.11);
            interpolatingDoubleTreeMap.put(2.58, 0.1);
            interpolatingDoubleTreeMap.put(3.32, 0.095);
            interpolatingDoubleTreeMap.put(3.64, 0.085);
            shooterWrist.coast();
            shooterWrist.setPower(0);
            shooterWristPIDController.setTolerance(ShooterConstants.SetpointThreshold);
        
            shooterWristPIDController.reset(shooterWrist.getPosition());
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
        shooterWristPIDController.setP(setpoint);

        double armPower = shooterWristPIDController.calculate(shooterWrist.getPosition());
        double armError = setpoint - shooterWrist.getPosition();
        armPower = armPower + armFeedforward.calculate(shooterWrist.getPositionInRadians(), shooterWrist.getVelocityInRadians());
        shooterWrist.setPower(armPower * 12);

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
        // return shooterWristPIDController.atGoal();
    }
}
