// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrapper;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.VisionConstants;

public class VisionWrapper {
  /** Creates a new VisionSubsystem. */
  
  PhotonCamera camera = new PhotonCamera("testCamera");
  PhotonPipelineResult result;
  PhotonPoseEstimator visionPoseEstimator;
  EstimatedRobotPose currentPoseEstimate;
 

  public VisionWrapper() {
    visionPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.ROBOT_TO_CAM);
  }
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }
  public Pose3d getCurrentPoseEstimate() {
    return currentPoseEstimate.estimatedPose;
  }
  public double getLatestTimeStamp() {
    return getLatestResult().getTimestampSeconds();
  }
  public Pose2d updatePoseEstimate() {
    try{
      currentPoseEstimate = visionPoseEstimator.update(getLatestResult()).get();
      return getCurrentPoseEstimate().toPose2d();
    }
    catch (Exception NoSuchElementException) {
      return null;
    }
  }
  
}


