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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.OneNote;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import frc.robot.subsystems.drive.TurnToSpeaker;
import frc.robot.subsystems.intake.IntakeNote;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.RunShooterWithIntake;

import frc.robot.subsystems.shooter.RunShooterWithPID;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterWrist.LowerShooter;
import frc.robot.subsystems.shooterWrist.PrepShooter;
// import frc.robot.subsystems.shooterWrist.RunShooterWristToSetpointWithCalculatedAngle;
import frc.robot.subsystems.shooterWrist.RunShooterWristWithJoystick;
import frc.robot.subsystems.shooterWrist.ShootNote;
import frc.robot.subsystems.shooterWrist.ShooterWrist;
import frc.robot.subsystems.shooterWrist.RaiseShooter;
import frc.robot.subsystems.shooterWrist.RaiseShooterWithMotionMagic;
import frc.robot.subsystems.shooterWrist.RaiseShooterWithPID;
import frc.robot.wrapper.VisionWrapper;

public class RobotContainer {

  // JOYSTICKS-----------------------------------------------------------------------------------------------------------------------

  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverJoystickPort);
  private final XboxController driverRaw = new XboxController(OperatorConstants.kDriverJoystickPort);
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorJoystickPort);

  // JOYSTICKS
  // END-------------------------------------------------------------------------------------------------------------------

  // SUBSYSTEMS----------------------------------------------------------------------------------------------------------------------

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterWrist shooterWrist = new ShooterWrist();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final VisionWrapper vision = new VisionWrapper();
  private final BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem();

  // SUBSYSTEMS
  // END------------------------------------------------------------------------------------------------------------------

  // TELEOP
  // COMMANDS-----------------------------------------------------------------------------------------------------------------

  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intakeSubsystem, operator);
  private final RunShooterWristWithJoystick runShooterWristWithJoystick = new RunShooterWristWithJoystick(shooterWrist,
      operator);
  private final RunShooterWithPID runShooterWithPID = new RunShooterWithPID(shooterSubsystem,
      3000, 3000);

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
  private final TurnToSpeaker turn = new TurnToSpeaker(drivetrain);
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

    oneNote = new OneNote(shooterSubsystem, shooterWrist, intakeSubsystem, drivetrain);
    twoNote = new PathPlannerAuto("Auton2NoteUpdateLeft");
    threeNoteLeft = new PathPlannerAuto("JawnAuto");
    threeNoteMiddle = new PathPlannerAuto("ThreeNoteMiddle");
    Auton1NoteUpdated = new PathPlannerAuto("Auton1NoteUpdated");
    drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.5, 5.5), new Rotation2d()));

    fieldAngle.HeadingController.setPID(7.5, 0, 0);
    fieldAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Registers commands in the path planner system, these can be used when running
   * paths
   */
  private void registerCommands() {
    NamedCommands.registerCommand("RunIntake", new RunIntake(intakeSubsystem, -0.7));
    NamedCommands.registerCommand("RunShooter", new RunShooter(shooterSubsystem, 0.7));
    // NamedCommands.registerCommand("WristToPosition", new RunShooterWristToSetpoint(shooterWrist, 0.158));
    NamedCommands.registerCommand("LowerShooter", new LowerShooter(shooterWrist));
    NamedCommands.registerCommand("Auto Shoot", new RaiseShooter(drivetrain, shooterWrist));
    NamedCommands.registerCommand("ShootNoteMiddle", new ShootNote(shooterWrist, shooterSubsystem, intakeSubsystem, 46.0/360, beamBreakSubsystem)); //tune
    NamedCommands.registerCommand("ShootNoteLeft", new ShootNote(shooterWrist, shooterSubsystem, intakeSubsystem, 46.0/360, beamBreakSubsystem)); //tune
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem, beamBreakSubsystem));
  }

  /**
   * This will configure the drive joystick bindings
   */
  private void configureBindings() {
    // Drivetrain
    // drivetrain.setDefaultCommand(drivetrainWithVision);

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed) // Drive
                                                                                                         // forward with
            // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * RobotConstants.kMaxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
        ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

    driver.y().whileTrue(turn);

    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    driver.leftBumper().onTrue(
        drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(new Translation2d(1.5, 5.5), new Rotation2d()))));

    // driver.rightBumper().whileTrue(
    // Commands.parallel(
    // drivetrain.driveAroundPoint(
    // -driver.getLeftY(),
    // -driver.getLeftX(),
    // drivetrain.getFuturePose()),
    // new PrepShooterForPoint(drivetrain, shooterWrist,
    // drivetrain.getFuturePose())));

    driver.rightBumper().whileTrue(
        // Commands.parallel(
            drivetrain.applyRequest(
                () -> fieldAngle.withVelocityX(-driver.getLeftY() * RobotConstants.kMaxSpeed)
                    .withVelocityY(-driver.getLeftX() * RobotConstants.kMaxSpeed)
                    .withTargetDirection(drivetrain.getDesiredAngle())
                    .withDeadband(RobotConstants.kMaxSpeed * 0.1)
                    .withRotationalDeadband(RobotConstants.kMaxAngularRate * 0.1)));
            // new PrepShooter(drivetrain, shooterWrist, drivetrain.getFuturePose())));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // zero
    operator.povUp().onTrue(new InstantCommand(shooterWrist::zeroEncoder));

    // operator.rightBumper().onTrue(runShooterWithPID);
    // operator.leftBumper().onTrue(new RunCommand(() -> {
    //   shooterSubsystem.setRPM(2000, 2000);
    // }, shooterSubsystem));
    // operator.rightBumper().whileTrue(new RunCommand(() -> {
    //   shooterSubsystem.setRPM(4000, 4000);
    // }, shooterSubsystem));

    operator.rightBumper().whileTrue(new RunShooter(shooterSubsystem, 0.7));
    //operator.rightBumper().onTrue(new ShootNote(shooterWrist, shooterSubsystem, intakeSubsystem, 45/360));

    
    // driver.povRight().toggleOnTrue(new RunShooterWithPID(shooterSubsystem,100, 500));
    driver.povLeft().onTrue(new RaiseShooterWithPID(shooterWrist, 53.0/360));

    // operator.b().onTrue(new RunShooterWristToSetpoint(sho/   eTrue(new RaiseShooter(drivetrain, shooterWrist));
    // operator.x().onTrue(new AutoShoot(shooterSubsystem, shooterWrist,
    // intakeSubsytem));
    // driver.a().onTrue(new RunShooterWristToSetpoint(shooterWrist, 0.005));
    driver.povDown().onTrue(new InstantCommand(drivetrain::setPoseStateToSpeaker));
    
    // operator.povLeft().onTrue(new RaiseShooterWithPID(shooterWrist, 46.0/360));
    // operator.a().whileTrue(new RaiseShooterWithMotionMagic(shooterWrist, 46.0/360));
    operator.y().onTrue(new RaiseShooter(drivetrain, shooterWrist));
    operator.b().whileTrue(new PrepShooter(drivetrain, shooterWrist));
    operator.a().whileTrue(new LowerShooter( shooterWrist));
    operator.povRight().toggleOnTrue(new RunCommand(() -> {
      shooterSubsystem.setRPM(2000, 2000);
    }, shooterSubsystem));

    operator.leftBumper().onTrue(new IntakeNote(intakeSubsystem, beamBreakSubsystem));
    // operator.x().onTrue(raiseShooterWithMotionMagic);

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
    intakeSubsystem.setDefaultCommand(runIntakeWithJoystick);
    shooterWrist.setDefaultCommand(runShooterWristWithJoystick);
    
    // shooterSubsystem.setDefaultCommand(new RunCommand(() -> {
    //   shooterSubsystem.setRPM(2000, 4000);
    // }, shooterSubsystem));
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
    return threeNoteLeft;

    // return twoNote;
  }
}
