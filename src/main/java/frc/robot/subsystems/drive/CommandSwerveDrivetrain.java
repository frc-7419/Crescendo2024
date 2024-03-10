package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.wrapper.VisionWrapper;
import org.photonvision.EstimatedRobotPose;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final VisionWrapper visionWrapper;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        visionWrapper = new VisionWrapper();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        visionWrapper = new VisionWrapper();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative,  // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    } else {
                        return false;
                    }
                }, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void setPoseState(double x, double y, double theta) {
        this.m_odometry.resetPosition(new Rotation2d(theta), m_modulePositions, new Pose2d(x, y, new Rotation2d(theta)));
    }

    public void setPoseStateToSpeaker() {
        this.m_odometry.resetPosition(new Rotation2d(0), m_modulePositions, new Pose2d(1.5, 5.5, new Rotation2d(0)));
    }

    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    public void periodic() {
        if (visionWrapper.updatePoseEstimate() != null) {
            EstimatedRobotPose estimatedPose = visionWrapper.updatePoseEstimate()[0];
            if (estimatedPose != null) {
                this.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
            }
        }
        SmartDashboard.putNumberArray("Future Pose", new Double[]{getFuturePose().getX(), getFuturePose().getY()});
        SmartDashboard.putNumber("Desired Angle", getDesiredAngle().getRadians() * (180 / Math.PI));
    }

    public Rotation2d getDesiredAngle() {
        Translation2d currentPose = getState().Pose.getTranslation();
        double desiredAngle = MathUtil.angleModulus(currentPose.minus(getFuturePose()).getAngle().getRadians());
        return new Rotation2d(desiredAngle);

    }

    public Translation2d getSpeakerPose() {
        boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
        Translation2d speakerPose = (isBlueAlliance) ? FieldConstants.BLUE_SPEAKER_TRANSLATION : FieldConstants.RED_SPEAKER_TRANSLATION;

        return speakerPose;
    }

    public Translation2d getFuturePose() {
        Translation2d speakerPose = getSpeakerPose();

        ChassisSpeeds robotVelocity = getCurrentRobotChassisSpeeds();
        double speakerDistance = getState().Pose.getTranslation().getDistance(speakerPose);

        double futureX = speakerPose.getX() - (robotVelocity.vxMetersPerSecond * (speakerDistance / RobotConstants.MAX_NOTE_VELOCITY));
        double futureY = speakerPose.getY() - (robotVelocity.vyMetersPerSecond * (speakerDistance / RobotConstants.MAX_NOTE_VELOCITY));

        return new Translation2d(futureX, futureY);
    }

    // public Command driveAroundPoint(double xVelocity, double yVelocity, Translation2d point) {

    //     double desiredAngle = getDesiredAngle();

    //     SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
    //     .withVelocityX(xVelocity)
    //     .withVelocityY(yVelocity)
    //     .withTargetDirection(new Rotation2d(desiredAngle));

    //     request.HeadingController.setPID(0.2, 0, 0);
    //     request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    //     SmartDashboard.putNumber("Request Desired Angle", desiredAngle * (180 / Math.PI));

    //     return this.applyRequest(() -> request);

    // }   

}
