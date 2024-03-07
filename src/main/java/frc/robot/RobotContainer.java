// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.OneNote;


import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RobotConstants.Action;


import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.intake.IntakeNote;
import frc.robot.subsystems.intake.Intake;

import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.intake.RunSerializer;

import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.RaiseShooter;
import frc.robot.subsystems.wrist.RunWristWithJoystick;

import frc.robot.subsystems.wrist.Wrist;
import frc.robot.wrapper.VisionWrapper;

public class RobotContainer {

  // JOYSTICKS-----------------------------------------------------------------------------------------------------------------------

  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverJoystickPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);

  // JOYSTICKS
  // END-------------------------------------------------------------------------------------------------------------------

  // SUBSYSTEMS----------------------------------------------------------------------------------------------------------------------

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final Shooter shooter = new Shooter();
  private final Wrist wrist = new Wrist();

  private final Intake intake = new Intake();
  private final VisionWrapper vision = new VisionWrapper();

  // SUBSYSTEMS
  // END------------------------------------------------------------------------------------------------------------------

  // TELEOP
  // COMMANDS-----------------------------------------------------------------------------------------------------------------

  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intake, operator);
  private final RunWristWithJoystick runwristWithJoystick = new RunWristWithJoystick(wrist,
      operator);

  // TELEOP COMMANDS
  // END-------------------------------------------------------------------------------------------------------------

  // SWERVE--------------------------------------------------------------------------------------------------------------------------

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(RobotConstants.kMaxSpeed * 0.05).withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.05); // Add
                                                                                                                    // a
                                                                                                                    // 5%
                                                                                                                    // deadband
  // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want
  // field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.FieldCentricFacingAngle fieldAngle = new SwerveRequest.FieldCentricFacingAngle();
  private final Telemetry logger = new Telemetry(RobotConstants.kMaxSpeed);

  // SWERVE
  // END----------------------------------------------------------------------------------------------------------------------

  // AUTONOMOUS----------------------------------------------------------------------------------------------------------------------

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final Command twoNote;
  private OneNote oneNote;
  private final Command threeNoteLeft;
  private final Command threeNoteMiddle;
  private final Command Auton1NoteUpdated;

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

    oneNote = new OneNote(shooter, wrist, intake, drivetrain);
    twoNote = new PathPlannerAuto("TwoNote");
    threeNoteLeft = new PathPlannerAuto("JawnAuto");
    threeNoteMiddle = new PathPlannerAuto("ThreeNoteMiddle");
    Auton1NoteUpdated = new PathPlannerAuto("Auton1NoteUpdated");
    drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.40, 5.5), new Rotation2d()));
    // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(0.68, 6.75), new
    // Rotation2d(Math.PI/3)));

    fieldAngle.HeadingController.setPID(7.5, 0, 0);
    fieldAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Registers commands in the path planner system, these can be used when running
   * paths
   */
  private void registerCommands() {
    NamedCommands.registerCommand("RevShooter", new RunShooter(shooter, 0.4));
    NamedCommands.registerCommand("RunShooter", new RunShooter(shooter, 0.7));
    NamedCommands.registerCommand("LowerShooter", new RaiseShooter(drivetrain, wrist, Action.STOW));
    NamedCommands.registerCommand("Auto Shoot", new RaiseShooter(drivetrain, wrist, Action.SPEAKER));
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intake));
 
  }

  /**
   * This will configure the drive joystick bindings
   */
  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed) // Drive
                                                                                                         // forward with
            // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * RobotConstants.kMaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
        ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));


    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver.leftBumper().onTrue(
        drivetrain
            .runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.5, 5.5), new Rotation2d()))));


    driver.rightBumper().whileTrue(
        // Commands.parallel(
        drivetrain.applyRequest(
            () -> fieldAngle.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                .withTargetDirection(drivetrain.getDesiredAngle())
                .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.1)));
    // new PrepShooter(drivetrain, wrist, drivetrain.getFuturePose())));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // zero
    // operator.povUp().onTrue(new InstantCommand(wrist::zeroEncoder));
    operator.rightBumper().whileTrue(new RunShooter(shooter, 0.7));
    operator.leftBumper().onTrue(new IntakeNote(intake));
    operator.y().onTrue(new RaiseShooter(drivetrain, wrist, Action.AMP));
    operator.povRight().onTrue(new RunCommand(() -> {
      shooter.setRPM(1000, 1000);
    }, shooter));
    operator.povLeft().onTrue(new RunCommand(() -> {
      shooter.setBothSpeed(0);
    }));  

    

    driver.povDown().onTrue(new InstantCommand(drivetrain::setPoseStateToSpeaker));

  }

  /**
   * Configures the auton chooser selections
   */
  public void configAutonSelection() {
    // autonChooser.setDefaultOption("Auton1NoteUpdated",Auton1NoteUpdated);
    // autonChooser.addOption("threeNoteMiddle", threeNoteMiddle);
    // autonChooser.addOption("threeNoteLeft", threeNoteLeft);
    // autonChooser.addOption("TwoNote(Tested)", twoNote);
  }

  /**
   * Sets the default commands to run during teleop
   */
  private void setDefaultCommands() {
    intake.setDefaultCommand(runIntakeWithJoystick);
    wrist.setDefaultCommand(runwristWithJoystick);
    // shooter.setDefaultCommand(new RunCommand(() -> {
    // shooter.setRPM(2000, 2000);
    // }, shooter));
  }

  /**
   * Gets the auton command selected by the user
   * 
   * @return selected autonomous command
   */
  public Command getAutonomousCommand() {
    // return autonChooser.getSelected();
    // return twoNote;
    // return threeNoteAuto;
    // return squareAuto;
    // return threeNoteLeft;
    // return threeNoteMiddle;
    // return new IntakeNote(intake);
    return new SequentialCommandGroup(
        new RunShooter(shooter, 1.0)
            .deadlineWith(new RaiseShooter(drivetrain, wrist, Action.SPEAKER))
            .deadlineWith(Commands.sequence(new WaitCommand(3), new RunSerializer(intake))).withTimeout(5));
    // return twoNote;
  }
}
