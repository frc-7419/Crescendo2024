// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShoot;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TurnToSpeaker;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RunIntake;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.shooter.RunShooter;
import frc.robot.subsystems.shooter.RunShooterVoltage;
import frc.robot.subsystems.shooter.RunShooterWithPID;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooterWrist.RunShooterWristToSetpoint;
// import frc.robot.subsystems.shooterWrist.RunShooterWristToSetpointWithCalculatedAngle;
import frc.robot.subsystems.shooterWrist.RunShooterWristWithJoystick;
import frc.robot.subsystems.shooterWrist.ShooterWrist;
import frc.robot.subsystems.shooterWrist.TurnShooterToTag;
import frc.robot.subsystems.shooterWrist.TurnShooterWithOdo;
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

  private final IntakeSubsystem intakeSubsytem = new IntakeSubsystem();
  private final VisionWrapper vision = new VisionWrapper(drivetrain);

  // private final IntakeWristSubsystem wristSubsystem = new
  // IntakeWristSubsystem(); //TODO figure out this situation

  // SUBSYSTEMS
  // END------------------------------------------------------------------------------------------------------------------

  // TELEOP
  // COMMANDS-----------------------------------------------------------------------------------------------------------------

  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intakeSubsytem, driver);
  private final RunShooterWristWithJoystick runShooterWristWithJoystick = new RunShooterWristWithJoystick(shooterWrist,
      operator);
  private final RunShooterWithPID runShooterWithPID = new RunShooterWithPID(shooterSubsystem,
      ShooterConstants.shooterPower, ShooterConstants.shooterPower);
  // private final RunIntakeWristWithJoystick runWristWithJoystick = new
  // RunIntakeWristWithJoystick(wristSubsystem, operator);
  // private final RunShooterToSetpoint runShooterToSetpoint = new
  // RunShooterToSetpoint(shooterSubsystem, 2000, 2000);

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
  // private final TurnToSpeaker turn = new TurnToSpeaker(drivetrain);
  private final Telemetry logger = new Telemetry(RobotConstants.kMaxSpeed);

  // SWERVE
  // END----------------------------------------------------------------------------------------------------------------------

  // AUTONOMOUS----------------------------------------------------------------------------------------------------------------------

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final Command testAuto;
  private final Command squareAuto;
  private final Command circleAuto;
  private final Command twoNote;
  private final Command threeNoteAuto;


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
    testAuto = new PathPlannerAuto("Test Auto");
    squareAuto = new PathPlannerAuto("Square Auto");
    circleAuto = new PathPlannerAuto("Circle Auto");
    twoNote = new PathPlannerAuto("TwoNote");
    threeNoteAuto = new PathPlannerAuto("Three Note Auto");
    drivetrain.seedFieldRelative(new Pose2d(new Translation2d(2, 4), new Rotation2d()));
  }

  /**
   * Registers commands in the path planner system, these can be used when running
   * paths
   */
  private void registerCommands() {
    NamedCommands.registerCommand("RunIntake", new RunIntake(intakeSubsytem, -0.7));
    NamedCommands.registerCommand("RunShooter", new RunShooter(shooterSubsystem, 0.7));
    NamedCommands.registerCommand("WristToPosition", new RunShooterWristToSetpoint(shooterWrist, 0.158));
    NamedCommands.registerCommand("ZeroWrist", new RunShooterWristToSetpoint(shooterWrist, 0.04));
    NamedCommands.registerCommand("Auto Shoot", new TurnShooterWithOdo(drivetrain, shooterWrist));
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
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driver.x().whileTrue(new TurnToSpeaker(drivetrain));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // zero
    operator.leftBumper().onTrue(new InstantCommand(shooterWrist::zeroEncoder));

    operator.rightBumper().whileTrue(new RunShooter(shooterSubsystem, 1));
    driver.rightBumper().whileTrue(new RunShooter(shooterSubsystem, 1));
    operator.b().onTrue(new RunShooterWristToSetpoint(shooterWrist, 0.17));
    operator.a().onTrue(new RunShooterWristToSetpoint(shooterWrist, 0.005));
    operator.y().whileTrue(runShooterWithPID);
    // operator.x().onTrue(new AutoShoot(shooterSubsystem, shooterWrist,
    // intakeSubsytem));
    driver.a().onTrue(new RunShooterWristToSetpoint(shooterWrist, 0.005));
    driver.povDown().onTrue(new InstantCommand(drivetrain::setPoseStateToSpeaker));
    driver.y().onTrue(new TurnShooterWithOdo(drivetrain, shooterWrist));
    operator.x().onTrue(new TurnShooterWithOdo(drivetrain, shooterWrist));

  }

  /**
   * Configures the auton chooser selections
   */
  public void configAutonSelection() {
    autonChooser.setDefaultOption("Test Auto", testAuto);
    autonChooser.addOption("Square Auto", squareAuto);
    autonChooser.addOption("Two Note Auto", twoNote);
    autonChooser.addOption("Circle Auto", circleAuto);
    autonChooser.addOption("Three Note", threeNoteAuto);
  }

  /**
   * Sets the default commands to run during teleop
   */
  private void setDefaultCommands() {
    intakeSubsytem.setDefaultCommand(runIntakeWithJoystick);
    // wristSubsystem.setDefaultCommand(runWristWithJoystick);
    shooterWrist.setDefaultCommand(runShooterWristWithJoystick);
  }

  /**
   * Gets the auton command selected by the user
   * 
   * @return selected autonomous command
   */
  public Command getAutonomousCommand() {
    // return autonChooser.getSelected();
    // return twoNote;
    return threeNoteAuto;
    // return squareAuto;
  }
}
