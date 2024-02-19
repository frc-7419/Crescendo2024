package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
public final class VisionConstants {
  public static final Transform3d ROBOT_TO_FRONT = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  public static final Transform3d ROBOT_TO_BACK = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //keshav

  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));
}
