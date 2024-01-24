// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.RunIntakeWithJoystick;
import frc.robot.subsystems.wrist.RunWristWithJoystick;
import frc.robot.subsystems.wrist.Wrist;

public class RobotContainer {
  /*
   * Prebuilt Swerve Stuff
   */
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Intake intake = new Intake();
  private final RunIntakeWithJoystick runIntakeWithJoystick = new RunIntakeWithJoystick(intake, driver);

  private final Wrist wrist = new Wrist();
  private final RunWristWithJoystick runWristWithJoystick = new RunWristWithJoystick(wrist, operator);
  
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05); // Add a 10% deadband
      // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /*
   * Autonomous Stuff
   */
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final Command testAuto = new PathPlannerAuto("Test Auto");
  private final Command squareAuto = new PathPlannerAuto("Square Auto");
  private final Command twoNoteAuto = new PathPlannerAuto("2 Note Auto");
  private final Command circleAuto = new PathPlannerAuto("Circle Auto");;

  private final List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
      new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
  new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
  new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
  );

  /**
   * This will configure the drive joystick bindings
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driver.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    driver.y().whileTrue(AutoBuilder.followPath(new PathPlannerPath(
      bezierPoints, 
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    )));    
  }

  /**
   * Configures the sendable chooser for auton
   */
  public void configAutonSelection() {
    autonChooser.setDefaultOption("Test Auto", testAuto);
    autonChooser.addOption("Square Auto", squareAuto);
    autonChooser.addOption("Two Note Auto", twoNoteAuto);
    //autonChooser.addOption("Three Note Auto", threeNoteAuto);
    autonChooser.addOption("Circle Auto", circleAuto);
  }
  /**
  * Creates a new RobotContainer
   */
  public RobotContainer() {
    configureBindings();
    configAutonSelection();
  }
  /**
   * Gets the auton command selected by the user
   * @return selected autonomous command
   */
  public Command getAutonomousCommand() {
    // return autonChooser.getSelected();
    return twoNoteAuto;
    // return squareAuto;
  }

  public void setDefaultCommands() {
    intake.setDefaultCommand(runIntakeWithJoystick);
    wrist.setDefaultCommand(runWristWithJoystick);
  }

}
