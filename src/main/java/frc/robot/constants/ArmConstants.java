package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public final class ArmConstants {
  public static final Measure<Angle> armOffset =Rotations.of(0.587).minus(Degrees.of(40));
  public static final double armGearing = 75;


}
