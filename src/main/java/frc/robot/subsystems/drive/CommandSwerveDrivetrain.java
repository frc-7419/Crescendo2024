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
import com.team7419.util.SwerveVoltageRequest;
import com.team7419.util.SysIdSignalLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.wrapper.VisionWrapper;
import org.photonvision.EstimatedRobotPose;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean doRejectUpdate;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        // this.getPigeon2().setYaw(0);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // this.getPigeon2().setYaw(0);
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
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0),
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

    public void updatePoseEstimate(){
        boolean useMegaTag2 = true;
        
        if(useMegaTag2){
            LimelightHelpers.SetRobotOrientation("limelight-tag",m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-tag");
            if(!(Math.abs(getPigeon2().getRate()) > 720 || mt2.tagCount == 0)){
                this.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }else{
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-tag");
            boolean doRejectUpdate = false;

            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
                if(mt1.rawFiducials[0].ambiguity > .7){
                    doRejectUpdate = true;
                } if(mt1.rawFiducials[0].distToCamera > 3){
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if(!doRejectUpdate){
                this.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                this.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }
    }

    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    public void periodic() {
        doRejectUpdate = false;
        // Update pose based on vision measurements
        updatePoseEstimate();
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
    private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

    private SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, SysIdSignalLogger.logState()), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))) ,
            null, 
            this));
    
    private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(false);

    private SysIdRoutine steerRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, SysIdSignalLogger.logState()), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(steerVoltageRequest.withVoltage(volts.in(Volts))) ,
            null, 
            this));

    private SysIdRoutine slipRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1)), null, null, SysIdSignalLogger.logState()), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> setControl(driveVoltageRequest.withVoltage(volts.in(Volts))) ,
            null, 
            this));
    
    public Command runDriveQuasistaticTest(Direction direction) {
        return driveRoutine.quasistatic(direction);
    }

    public Command runDriveDynamicTest(Direction direction) {
        return driveRoutine.dynamic(direction);
    }
    public Command runSteerQuasistaticiTest(Direction direction) {
        return steerRoutine.quasistatic(direction);
    }
    public Command runSteerDynamicTest(Direction direction) {
        return steerRoutine.dynamic(direction);
    }

    public Command runSlipTest() {
        return slipRoutine.quasistatic(Direction.kForward);
    }


 

}
