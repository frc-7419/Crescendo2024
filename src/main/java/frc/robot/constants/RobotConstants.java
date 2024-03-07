package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class RobotConstants {
  public static final double TalonFXTicksPerRotation = 2048;
  public static final double kMaxSpeed = 5.2;// 4.5 meters per second desired top speed
  public static final double kMaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  public static final double kTrackWidth = 0.6858; // meters

  public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
  public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  // public static final String currentAllianceColor = Robot.getAllianceColor();
  // public static final Alliance currentAlliance = Robot.getAlliance();
  // public static final String currentAllianceSide = Robot.getAllianceSide();

  public static final double mainArmGearRatio = 100;
  // arbitrary until we mount camera
  public static final Transform3d kCameraToRobot = new Transform3d();

  //TODO get intake height, this just an estimate
  public static final double shooterWristHeight = 8.37 * 0.0254;

  public static final double autoScoreDelaySeconds = 0.5;

  public static final double joystickDeadZone = 0.07;

  public static final double maxVoltage = 12;
  public static final double voltageCompSaturation = 11;

  public static final double MAX_NOTE_VELOCITY = 20.0;

  public final class IntakeWristConstants{
    public static final double wristPower = 0.2;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final ArmFeedforward wristFeedForward = new ArmFeedforward(kS, kG, kV, kA);
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;
    public static final Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    public static final double SetpointThreshold = 0;
  }

  public final class ShooterConstants{
    public static final double shooterPower = 0.8;
    public static final double wristPower = 0.2;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0.0075;
    public static final double kA = 0;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 2000;
    public static final double maxAcceleration = 0;
    public static final double tolerance = 500;
    public static final Constraints Constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
    public static final double SetpointThreshold = 1.0/360;
    
    public static final int topShooterStallLimit = 20;
    public static final int topShooterFreeLimit = 20;
    public static final int bottomShooterStallLimit = 20;
    public static final int bottomShooterFreeLimit = 20;
  }
  public static enum Action {
    SPEAKER(),
    STOW(),
    AMP();

    private Action() {
    }
  }
}
