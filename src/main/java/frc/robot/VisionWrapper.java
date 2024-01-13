// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import javax.print.attribute.HashAttributeSet;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;

public class VisionWrapper {
  /** Creates a new VisionSubsystem. */
  
  PhotonCamera camera = new PhotonCamera("testCamera");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  private Map<Integer, Pose3d> poses = new HashMap<Integer, Pose3d>();
  private PhotonPipelineResult result;
  private double resultTimeStamp;
  private double previousTimeStamp;
 

  public VisionWrapper() {
    // aprilTagFieldLayout = new AprilTagFieldLayout("2024-crescendo.json");
    poses.put(1, new Pose3d(1,1,1, new Rotation3d(0,0,Units.degreesToRadians(180))));
  }

  
  public double getTimestampSeconds() {
    return result.getTimestampSeconds();
  }
  public Pose2d getVisionUpdate() {
    result = camera.getLatestResult();
    resultTimeStamp = result.getTimestampSeconds();

    if (result.hasTargets() && resultTimeStamp != previousTimeStamp) {
      previousTimeStamp = resultTimeStamp;
      PhotonTrackedTarget target = result.getBestTarget();
      int fiducialId = target.getFiducialId();

      if (target.getPoseAmbiguity() <= VisionConstants.visionAmbiguityThreshold) {
        Pose3d targetPose = poses.get(fiducialId);
        Transform3d camToTargetTrans = target.getBestCameraToTarget();
        Pose3d camPose =
            targetPose.transformBy(camToTargetTrans.inverse()); // this lines uses where the target
        // is on the field physically and
        // gets the camera pose
        return (camPose.transformBy(RobotConstants.kCameraToRobot).toPose2d());
       }
     
    }
     return null;
  }

  
}


