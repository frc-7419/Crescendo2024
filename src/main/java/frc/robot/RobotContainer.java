// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GoToAmpPosition;
import frc.robot.commands.GoToFunnelPosition;
import frc.robot.commands.GoToShootPosition;
import frc.robot.commands.OneNote;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.RunClimberWithJoystick;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TurnToAmp;
import frc.robot.subsystems.drive.TurnToSpeaker;
import frc.robot.subsystems.intake.IntakeNote;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.intake.RunSerializer;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.RunShooterWithPID;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterWrist.*;
import frc.robot.wrapper.VisionWrapper;

public class RobotContainer {

        // JOYSTICKS-----------------------------------------------------------------------------------------------------------------------

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverJoystickPort);
        private final CommandXboxController operator = new CommandXboxController(
                        OperatorConstants.kOperatorJoystickPort);

        // JOYSTICKS
        // END-------------------------------------------------------------------------------------------------------------------

        // SUBSYSTEMS----------------------------------------------------------------------------------------------------------------------
        private final XboxController driverRaw = new XboxController(OperatorConstants.kDriverJoystickPort);
        private final XboxController operatorRaw = new XboxController(OperatorConstants.kOperatorJoystickPort);
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final ShooterWrist shooterWrist = new ShooterWrist();
        private final Climber climber = new Climber();

        private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(beamBreakSubsystem);
        private final LEDSubsystem ledSubsystem = new LEDSubsystem(beamBreakSubsystem, shooterSubsystem,
                        intakeSubsystem, shooterWrist, operatorRaw);
        private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intakeSubsystem,
                        operator);
        private final RunClimberWithJoystick runClimberWithJoystick = new RunClimberWithJoystick(climber, operator);

        // SUBSYSTEMS
        // END------------------------------------------------------------------------------------------------------------------

        // TELEOP
        // COMMANDS-----------------------------------------------------------------------------------------------------------------
        private final VisionWrapper vision = new VisionWrapper();
        private final RunShooterWristWithJoystick runShooterWristWithJoystick = new RunShooterWristWithJoystick(
                        shooterWrist,
                        operator);
        private final RunShooterWithPID runShooterWithPID = new RunShooterWithPID(shooterSubsystem, 5500, 3000);

        // TELEOP COMMANDS
        // END-------------------------------------------------------------------------------------------------------------

        // SWERVE--------------------------------------------------------------------------------------------------------------------------

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(RobotConstants.kMaxSpeed * 0.05)
                        .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.05); // Add
        // a
        // 5%
        // deadband
        // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want
        // field-centric
        // driving in open loop
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final TurnToSpeaker turn = new TurnToSpeaker(drivetrain);
        private final Telemetry logger = new Telemetry(RobotConstants.kMaxSpeed);
        private final SendableChooser<Command> autonChooser = new SendableChooser<>();

        // SWERVE
        // END----------------------------------------------------------------------------------------------------------------------

        // AUTONOMOUS----------------------------------------------------------------------------------------------------------------------
        // private final Command twoNote;
        private final OneNote oneNote;
        private final Command oneNoteRight;
        private final Command oneNoteLeft;
        private final Command threeNoteMiddleLeft;
        private final Command threeNoteLeft;
        private final Command threeNoteMiddle;
        private final Command threeNoteRight;
        private final Command threeNoteMidline;
        private final Command fourNoteMiddle;
        private final Command fourhalfMiddle;
        private final Command fiveNoteMiddle;
        // private final Command Auton1NoteUpdated;
        // private final Command poleAuto;
        private final Command funnyAuto;
        private final SwerveRequest.FieldCentricFacingAngle fieldAngle = new SwerveRequest.FieldCentricFacingAngle();

        // AUTONOMOUS
        // END------------------------------------------------------------------------------------------------------------------

        /**
         * Creates a new RobotContainer
         */
        public RobotContainer() {
                registerCommands();
                configureBindings();
                configAutonSelection();
                setDefaultCommands();
                SmartDashboard.putNumber("top RPM", 1000);
                SmartDashboard.putNumber("bottom RPM", 1000);

                oneNote = new OneNote(shooterSubsystem, shooterWrist, intakeSubsystem, drivetrain);
                oneNoteRight = new PathPlannerAuto("OneNoteRight");
                oneNoteLeft = new PathPlannerAuto("OneNoteLeft");
                threeNoteMiddleLeft = new PathPlannerAuto("ThreeNoteMiddleLeft");
                // twoNote = new PathPlannerAuto("TwoNote");
                threeNoteLeft = new PathPlannerAuto("ThreeNoteLeft");
                threeNoteMiddle = new PathPlannerAuto("ThreeNoteMiddle");
                threeNoteRight = new PathPlannerAuto("FourNoteRight");
                fourNoteMiddle = new PathPlannerAuto("FourNoteMiddleOp");
                fiveNoteMiddle = new PathPlannerAuto("FiveNoteMiddleOp");
                fourhalfMiddle = new PathPlannerAuto("FourHalfNote");
                funnyAuto = new PathPlannerAuto("FunnyAuto");
                threeNoteMidline = new PathPlannerAuto("ThreeMidline");
                // Auton1NoteUpdated = new PathPlannerAuto("Auton1NoteUpdated");
                // poleAuto = new PathPlannerAuto("1m pole");

                // middle

                // !!! WE ARE USING START POSITION ON PATHPLANNER, NO NEED TO SEED !!! //

                // blue
                // drivetrain.seedFieldRelative( new Pose2d(new Translation2d(1.5, 5.5), new
                // Rotation2d(0)));

                // red
                // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(15.25, 5.5), new
                // Rotation2d(Math.PI)));

                // ******************************************************************************************************
                // //

                // left

                // blue
                // drivetrain.seedFieldRelative( new Pose2d(new Translation2d(0.68, 6.75), new
                // Rotation2d(Math.PI/3)));

                // red
                // drivetrain.seedFieldRelative( new Pose2d(new Translation2d(15.68, 6.75), new
                // Rotation2d(-Math.PI/3)));

                // ******************************************************************************************************
                // //

                // right

                // blue
                // drivetrain.seedFieldRelative( new Pose2d(new Translation2d(0.68, 4.25), new
                // Rotation2d(-Math.PI/3)));

                // red
                // drivetrain.seedFieldRelative( new Pose2d(new Translation2d(15.68, 4.25), new
                // Rotation2d(Math.PI/3)));

                fieldAngle.HeadingController.setPID(7.5, 0, 0);
                fieldAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        }

        /**
         * Registers commands in the path planner system, these can be used when running
         * paths
         */
        private void registerCommands() {
                NamedCommands.registerCommand("RunIntake", new RunIntake(intakeSubsystem, 0.7));
                NamedCommands.registerCommand("RunShooter", new RunShooter(shooterSubsystem, 0.7));
                // NamedCommands.registerCommand("LowerShooter", new
                // LowerShooter(shooterWrist));
                NamedCommands.registerCommand("Auto Shoot", new RaiseShooterWithVision(drivetrain, shooterWrist));
                NamedCommands.registerCommand("ShootNoteMid",
                                new ShootNote(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem, 66.0 / 360));
                NamedCommands.registerCommand("ShootNoteMidFar",
                                new ShootNote(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem, 39.0 / 360));
                NamedCommands.registerCommand("ShootNoteFar",
                                new ShootNoteFar(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem,
                                                37.5 / 360));
                NamedCommands.registerCommand("ShootNoteDown",
                                new ShootNote(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem, 33.0 / 360)); // change
                                                                                                                         // this
                NamedCommands.registerCommand("ShootNoteLeft",
                                new ShootNote(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem, 62.0 / 360));
                NamedCommands.registerCommand("ShootNoteRight",
                                new ShootNote(shooterWrist, shooterSubsystem, drivetrain, intakeSubsystem, 62.0 / 360));
                NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem, driverRaw, operatorRaw));
                NamedCommands.registerCommand("RevShooter",
                                new RunCommand(() -> {
                                        shooterSubsystem.setRPM(3500, 3500);
                                }, shooterSubsystem));
        }

        public Command funnelRotation() {
                boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
                double angle = (isBlueAlliance ? 20 : -20) * Math.PI / 180.0;
                return drivetrain.applyRequest(
                                () -> fieldAngle.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                                                .withTargetDirection(new Rotation2d(angle))
                                                .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                                                .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.1));
        }

        /**
         * This will configure the drive joystick bindings
         */
        private void configureBindings() {
                // Drivetrain
                // drivetrain.setDefaultCommand(drivetrainWithVision);

                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed) // Drive
                                                // forward with
                                                // negative Y (forward)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                                                .withRotationalRate(
                                                                -driver.getRightX() * RobotConstants.kMaxAngularRate) // Drive
                                                                                                                      // counterclockwise
                                                                                                                      // with
                                // negative X (left)
                                ));

                driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

                driver.y().whileTrue(turn);

                driver.b().whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                driver.leftBumper().onTrue(
                                drivetrain
                                                .runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(
                                                                new Translation2d(1.5, 15.5), new Rotation2d()))));

                // driver.rightBumper().whileTrue(
                // drivetrain.applyRequest(
                // () -> fieldAngle.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                // .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                // .withTargetDirection(new Rotation2d(Math.atan2(-driver.getLeftX(),
                // -driver.getLeftY())))
                // .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                // .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.1)).until(() ->
                // beamBreakSubsystem.frontBeamBreakIsTriggered())
                // );
                driver.x().whileTrue(
                                drivetrain.applyRequest(
                                                () -> fieldAngle.withVelocityX(
                                                                -driver.getLeftY() * RobotConstants.kMaxSpeed)
                                                                .withVelocityY(-driver.getLeftX()
                                                                                * RobotConstants.kMaxSpeed)
                                                                .withTargetDirection(new Rotation2d(0))
                                                                .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                                                                .withRotationalDeadband(
                                                                                RobotConstants.kMaxAngularRate * 0.1)));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);

                operator.rightBumper().whileTrue(new RunShooter(shooterSubsystem, 1.0));
                operator.leftBumper().onTrue(new IntakeNote(intakeSubsystem, driverRaw, operatorRaw));
                // operator.y().whileTrue(new RaiseShooter(drivetrain, shooterWrist, 60.0 /
                // 360));
                // operator.a().whileTrue(new RaiseShooter(drivetrain, shooterWrist, 39.0 /
                // 360));

                // funneling
                // operator.b().whileTrue(
                // new ParallelCommandGroup(
                // new RaiseShooter(drivetrain, shooterWrist, 59.0/360),
                // new RunCommand(() -> {
                // shooterSubsystem.setRPM(4900, 4900);
                // }, shooterSubsystem)));

                // operator.x().whileTrue(new RaiseShooterWithVision(drivetrain, shooterWrist));

                operator.povRight().whileTrue(new RunCommand(() -> {
                        shooterSubsystem.setRPM(1250, 1250);
                }, shooterSubsystem));

                operator.povUp().whileTrue(new RunCommand(() -> {
                        shooterSubsystem.setRPM(1050, 1200);
                }, shooterSubsystem));

                operator.povLeft().onTrue(new RunCommand(() -> {
                        shooterSubsystem.setBothVoltage(0);
                        shooterSubsystem.brake();
                }).withTimeout(1));

                driver.povUp().whileTrue(new GoToShootPosition(drivetrain));
                driver.povLeft().whileTrue(new GoToAmpPosition(drivetrain));
                driver.povRight().whileTrue(new GoToFunnelPosition(drivetrain));

                driver.a().and(driver.povUp()).whileTrue(drivetrain.runDriveQuasistaticTest(Direction.kForward));
                driver.a().and(driver.povDown()).whileTrue(drivetrain.runDriveQuasistaticTest(Direction.kReverse));

                driver.x().and(driver.povUp()).whileTrue(drivetrain.runDriveDynamicTest(Direction.kForward));
                driver.x().and(driver.povDown()).whileTrue(drivetrain.runDriveDynamicTest(Direction.kReverse));

                driver.y().and(driver.povUp()).whileTrue(drivetrain.runSteerQuasistaticiTest(Direction.kForward));
                driver.y().and(driver.povDown()).whileTrue(drivetrain.runSteerQuasistaticiTest(Direction.kReverse));

                driver.b().and(driver.povUp()).whileTrue(drivetrain.runSteerDynamicTest(Direction.kForward));
                driver.b().and(driver.povDown()).whileTrue(drivetrain.runSteerDynamicTest(Direction.kReverse));

                driver.rightBumper().whileTrue(funnelRotation());
        }

        /**
         * Configures the auton chooser
         * selections
         */
        public void configAutonSelection() {
                autonChooser.setDefaultOption("Four Note Middle", fourNoteMiddle);
                autonChooser.addOption("Three Note Middle", threeNoteMiddle);
                autonChooser.addOption("Three Note Left", threeNoteLeft);
                autonChooser.addOption("Three Note Midline", threeNoteMidline);
                autonChooser.addOption("Five Note Middle", fiveNoteMiddle);
                SmartDashboard.putData(autonChooser);
        }

        public double getRotation() {
                boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
                double angle = (isBlueAlliance) ? 180 : 0;

                return angle;
        }

        public Command autoFaceNote() {

                return drivetrain.applyRequest(
                                () -> fieldAngle.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                                                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                                                .withTargetDirection(new Rotation2d(
                                                                Math.atan2(-driver.getLeftX(), -driver.getLeftY())))
                                                .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                                                .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.1));
        }

        /**
         * Sets the default commands to run during teleop
         */
        private void setDefaultCommands() {
                intakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
                shooterWrist.setDefaultCommand(runShooterWristWithJoystick);
                climber.setDefaultCommand(runClimberWithJoystick);
        }

        /**
         * Gets the auton command selected by the user
         *
         * @return selected autonomous command
         */
        public Command getAutonomousCommand() {
                // return autonChooser.getSelected();
                // return oneNoteRight;
                // return oneNoteLeft;
                // return oneNote;
                // return threeNoteAuto;
                // return threeNoteRight;
                // return threeNoteMiddle;
                // return threeNoteMiddleLeft;
                return fourNoteMiddle;
                // return funnyAuto;
                // return fiveNoteMiddle;
                // return fourhalfMiddle;

        }
}
