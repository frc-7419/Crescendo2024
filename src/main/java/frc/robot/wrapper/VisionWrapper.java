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

import frc.robot.constants.VisionConstants;

public class VisionWrapper {
  
  PhotonCamera camera = new PhotonCamera("testCamera");
  PhotonCamera keshav = new PhotonCamera("keshavCamera");
  PhotonPipelineResult result;
  //pos estmatos
  PhotonPoseEstimator forwardPoseEstimator;
  PhotonPoseEstimator backPoseEstimator;
  PhotonPoseEstimator leftPoseEstimator;
  PhotonPoseEstimator rightPoseEstimator;
 

  public VisionWrapper() {
    forwardPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, VisionConstants.ROBOT_TO_FRONT);
    backPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, keshav, VisionConstants.ROBOT_TO_BACK);
  }
  public PhotonPipelineResult[] getLatestResults() {
    PhotonPipelineResult result1 = camera.getLatestResult();
    PhotonPipelineResult result2 = keshav.getLatestResult();
    return new PhotonPipelineResult[] {result1,result2,null,null};
  }
  public EstimatedRobotPose[] updatePoseEstimate() {
    EstimatedRobotPose[] out = new EstimatedRobotPose[4];
    Optional<EstimatedRobotPose> frontPose = forwardPoseEstimator.update();
    Optional<EstimatedRobotPose> backPose = backPoseEstimator.update();
    //Optional<EstimatedRobotPose> leftPose = leftPoseEstimator.update();
    //Optional<EstimatedRobotPose> rightPose = rightPoseEstimator.update();
    if (frontPose.isPresent()) {
      out[0] = frontPose.get();
    }
    if (backPose.isPresent()) {
      out[1] = backPose.get();
    }
    return out;

  }
  
}


