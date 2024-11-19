// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

public class NoteVisionSubsystem extends SubsystemBase {
  private double kNoteCameraHeight = Units.inchesToMeters(0); // move to constants and fix **
  private double kNoteCameraAngle = Units.degreesToRadians(0);

  public NoteVisionSubsystem() {

  }

  public LimelightTarget_Detector[] getTargets() {
    LimelightResults results = LimelightHelpers.getLatestResults("note");
    return results.targetingResults.targets_Detector;
  }

  public LimelightTarget_Detector[] getTarget() {
    LimelightResults results = LimelightHelpers.getLatestResults("note");
    return results.targetingResults.targets_Detector;
  }

  public Pose2d getNotePose() {
    LimelightResults results = LimelightHelpers.getLatestResults("note");
    double ty = 0;
    double tx = 0;
    double calcY = 0;
    double calcX = 0;

    if (results.targetingResults.targets_Detector.length > 0) {
      LimelightTarget_Detector detection = results.targetingResults.targets_Detector[0];
      String className = detection.className;
      ty = detection.ty;
      tx = detection.tx;

      if (className.equals("note")) {
        double targetHeight = Units.inchesToMeters(1); // 1 inch tall note

        double yAngle = kNoteCameraAngle + Units.degreesToRadians(ty);
        calcY = (targetHeight - kNoteCameraAngle) / Math.tan(yAngle);

        double xAngle = kNoteCameraAngle + Units.degreesToRadians(tx);
        calcX = calcY * Math.tan(xAngle);
      }
    }
    return new Pose2d(tx, ty, new Rotation2d(calcX, calcY));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
