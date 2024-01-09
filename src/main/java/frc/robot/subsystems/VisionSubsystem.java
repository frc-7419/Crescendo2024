// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private AprilTagFieldLayout aprilTagFieldLayout;
  public VisionSubsystem() throws IOException {
    aprilTagFieldLayout = new AprilTagFieldLayout("2024-crescendo.json");
  }

  public Optional<Pose3d> getBlueAmp(){
    Optional<Pose3d> blueAmp = aprilTagFieldLayout.getTagPose(6);
    return blueAmp;
  }
    public Optional<Pose3d> getRedAmp(){
    Optional<Pose3d> redAmp = aprilTagFieldLayout.getTagPose(5);
    return redAmp;
  }


  public Optional<Pose3d> getRedSpeakerMiddle(){
    Optional<Pose3d> redSpeakerMiddle = aprilTagFieldLayout.getTagPose(4);
    return redSpeakerMiddle;
  }
  public Optional<Pose3d> getRedSpeakerRight(){
    Optional<Pose3d> redSpeakerRight = aprilTagFieldLayout.getTagPose(3);
    return redSpeakerRight;
  }
  public Optional<Pose3d> getRedSourceRight(){
    Optional<Pose3d> redSourceRight = aprilTagFieldLayout.getTagPose(9);
    return redSourceRight;
  }
  public Optional<Pose3d> getRedSourceLeft(){
    Optional<Pose3d> redSourceLeft = aprilTagFieldLayout.getTagPose(10);
    return redSourceLeft;
  }
    public Optional<Pose3d> getBlueSourceRight(){
    Optional<Pose3d> blueSourceRight = aprilTagFieldLayout.getTagPose(1);
    return blueSourceRight;
  }
  public Optional<Pose3d> getBlueSourceLeft(){
    Optional<Pose3d> blueSourceLeft = aprilTagFieldLayout.getTagPose(2);
    return blueSourceLeft;
  }
    public Optional<Pose3d> getBlueSpeakerMiddle(){
    Optional<Pose3d> blueSpeakerMiddle = aprilTagFieldLayout.getTagPose(7);
    return blueSpeakerMiddle;
  }
    public Optional<Pose3d> getBlueSpeakerLeft(){
    Optional<Pose3d> blueSpeakerLeft = aprilTagFieldLayout.getTagPose(8);
    return blueSpeakerLeft;
  }
  public Optional<Pose3d> getRedStage1(){
    Optional<Pose3d> redStage1 = aprilTagFieldLayout.getTagPose(11);
    return redStage1;
  }
    public Optional<Pose3d> getRedStage2(){
    Optional<Pose3d> redStage2 = aprilTagFieldLayout.getTagPose(12);
    return redStage2;
  }
    public Optional<Pose3d> getBlueStage1(){
    Optional<Pose3d> blueStage1 = aprilTagFieldLayout.getTagPose(14);
    return blueStage1;
  }
     
  public Optional<Pose3d> getBlueStage2(){
    Optional<Pose3d> blueStage2 = aprilTagFieldLayout.getTagPose(15);
    return blueStage2;
  }
      public Optional<Pose3d> getBlueStage3(){
    Optional<Pose3d> blueStage3 = aprilTagFieldLayout.getTagPose(16);
    return blueStage3;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


// 1: blue source right / 
// 2: blue source left / 
// 3: red speaker right / 
// 4: red speaker middle / 
// 5: red amp /
// 6: blue amp /
// 7: blue speaker middle / 
// 8: blue speaker left / 
// 9: red source right /
// 10: red source left /
// 11: red stage 1 /
// 12: red stage 2 /
// 13: red stage 3 /
// 14: blue stage 1 /
// 15: blue stage 2 /
// 16: blue stage 3 /