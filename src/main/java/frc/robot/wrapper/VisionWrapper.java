// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.wrapper;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import frc.robot.constants.VisionConstants;
// /**
//  * PhotonVision is handled in this class
//  */
// public class VisionWrapper {

//   private final PhotonCamera frontCam;
//   private final PhotonCamera backCam;
//   private final PhotonPoseEstimator frontPoseEstimator;
//   private final PhotonPoseEstimator backPoseEstimator;
//   private PhotonPipelineResult result;

//   public VisionWrapper() {
//     frontCam = new PhotonCamera("frontCam");
//     backCam = new PhotonCamera("backCam");
//     frontPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, VisionConstants.ROBOT_TO_FRONT);
//     backPoseEstimator = new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam, VisionConstants.ROBOT_TO_BACK);
//   }
//   /**
//    * Uses the cameras to estimate the pose of the robot ont he field
//    * @return The poses esimated by the two cameras
//    */
//   public EstimatedRobotPose[] updatePoseEstimate() {
//     EstimatedRobotPose[] out = new EstimatedRobotPose[2];
//     Optional<EstimatedRobotPose> frontPose = frontPoseEstimator.update();
//     Optional<EstimatedRobotPose> backPose = backPoseEstimator.update();
//     // Optional<EstimatedRobotPose> leftPose = leftPoseEstimator.update();
//     // Optional<EstimatedRobotPose> rightPose = rightPoseEstimator.update();
//     frontPose.ifPresent(estimatedRobotPose -> out[0] = estimatedRobotPose);
//     backPose.ifPresent(estimatedRobotPose -> out[1] = estimatedRobotPose);
//     return out;
//   }

// }
