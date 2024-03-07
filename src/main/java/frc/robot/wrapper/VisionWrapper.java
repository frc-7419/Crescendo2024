// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrapper;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;

/**
 * PhotonVision is handled in this class
 */
public class VisionWrapper extends SubsystemBase {

  private final PhotonCamera frontCam;
  // private final PhotonCamera backCam;
  private final PhotonPoseEstimator frontPoseEstimator;
  // private final PhotonPoseEstimator backPoseEstimator;
  private PhotonPipelineResult result;

  public VisionWrapper() {
    frontCam = new PhotonCamera("frontCam");
    // backCam = new PhotonCamera("backCam");
    frontPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, VisionConstants.ROBOT_TO_FRONT);
    // backPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
    // VisionConstants.ROBOT_TO_BACK);
  }

  /**
   * Uses the cameras to estimate the pose of the robot ont he field
   * 
   * @return The poses esimated by the two cameras
   */
  public EstimatedRobotPose[] updatePoseEstimate() {
    EstimatedRobotPose[] out = new EstimatedRobotPose[2];
    Optional<EstimatedRobotPose> frontPose = frontPoseEstimator.update();
    // Optional<EstimatedRobotPose> backPose = backPoseEstimator.update();
    // Optional<EstimatedRobotPose> leftPose = leftPoseEstimator.update();
    // Optional<EstimatedRobotPose> rightPose = rightPoseEstimator.update();
    frontPose.ifPresent(estimatedRobotPose -> out[0] = estimatedRobotPose);
    // backPose.ifPresent(estimatedRobotPose -> out[1] = estimatedRobotPose);
    return out;
  }

  public double headingToTag(int id) {
    PhotonPipelineResult[] results = latestResults();
    PhotonTrackedTarget best = null;
    double bestAmb = 2.0;
    for (PhotonPipelineResult result : results) {
      try {
        if (result.hasTargets()) {
          System.out.println(result.getBestTarget());
          if (result.getBestTarget().getFiducialId() == id) {
            if (result.getBestTarget().getPoseAmbiguity() < bestAmb) {
              bestAmb = result.getBestTarget().getPoseAmbiguity();
              best = result.getBestTarget();
            }
          }
        }
      } catch (NullPointerException e) {
        e.printStackTrace();
      }

    }
    if (best == null)
      // WHO EVER KEEPS CHANGING THIS TO Double.MINVALUE THIS IS WHAT IS BREAKING THE CODE. THE DRIVE TRAIN BUGS OUT WHEN GIVEN THIS VALUE AND BREAKS EVERYTHING
      return 0;
    return best.getYaw();
  }

  public double distanceToTag(int id) {
    var result = frontCam.getLatestResult();
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(10);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(6.5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

    if (result.hasTargets()) {
      // First calculate range
      double range = PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch()));
      return range;
    } else {
      return Integer.MAX_VALUE;
    }
  }

  public PhotonPipelineResult[] latestResults() {
    PhotonPipelineResult frontResult = frontCam.getLatestResult();
    // PhotonPipelineResult backResult = backCam.getLatestResult();
    return new PhotonPipelineResult[] { frontResult };
  }

 


}
