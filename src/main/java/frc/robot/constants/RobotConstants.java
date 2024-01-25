package frc.robot.constants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

public final class RobotConstants {
  public static final double TalonFXTicksPerRotation = 2048;

  public static final double kTrackWidth = 0.6858; // meters

  public static final double kWheelRadius = 3 * 0.0254; // inches TO centimeters conversion
  public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;
  // public static final String currentAllianceColor = Robot.getAllianceColor();
  // public static final Alliance currentAlliance = Robot.getAlliance();
  // public static final String currentAllianceSide = Robot.getAllianceSide();

  public static final double mainArmGearRatio = 100;
  // arbitrary until we mount camera
  public static final Transform3d kCameraToRobot = new Transform3d();

  public static double autoScoreDelaySeconds = 0.5;

  public static double joystickDeadZone = 0.07;

  public static double maxVoltage = 12;
  public static double voltageCompSaturation = 11;

  public final class WristConstants{

    public static final double wristPower = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final ArmFeedforward wristFeedForward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;
    public static double SetpointThreshold;
  }

  public final class ShooterConstants{

    public static final double wristPower = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0.0075;
    public static final double kA = 0;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double maxVelocity = 0;
    public static final double maxAcceleration = 0;
    public static double SetpointThreshold;
  }
}
