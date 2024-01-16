// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrapper;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.VisionConstants;

public class VisionWrapper {

  PhotonCamera frontCam = new PhotonCamera("frontCam");
  PhotonCamera backCam = new PhotonCamera("backCam");
  PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCam, VisionConstants.ROBOT_TO_FRONT);
  PhotonPoseEstimator backPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT,
      PoseStrategy.CLOSEST_TO_REFERENCE_POSE, backCam, VisionConstants.ROBOT_TO_BACK);
  PhotonPipelineResult result;

  public VisionWrapper() {
  }

  public EstimatedRobotPose[] updatePoseEstimate() {
    EstimatedRobotPose[] out = new EstimatedRobotPose[2];
    Optional<EstimatedRobotPose> frontPose = frontPoseEstimator.update();
    Optional<EstimatedRobotPose> backPose = backPoseEstimator.update();
    // Optional<EstimatedRobotPose> leftPose = leftPoseEstimator.update();
    // Optional<EstimatedRobotPose> rightPose = rightPoseEstimator.update();
    if (frontPose.isPresent()) {
      out[0] = frontPose.get();
    }
    if (backPose.isPresent()) {
      out[1] = backPose.get();
    }
    return out;
  }

}
