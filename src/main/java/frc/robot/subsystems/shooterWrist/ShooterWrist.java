// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.DeviceIDs.CanIds;

public class ShooterWrist extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX armMotor;
  private DutyCycleOut dutyCycleOut;
  private DutyCycleEncoder encoder;

    public ShooterWrist() {
    armMotor = new TalonFX(CanIds.shooterWrist.id, "Ryan Biggee");
    dutyCycleOut = new DutyCycleOut(0);
    armMotor.setInverted(false);
    armMotor.setPosition(0);
    encoder = new DutyCycleEncoder(0 );
    encoder.setPositionOffset(ArmConstants.armOffset);
  }

  public double rotationToRadians(double rotations){
    return rotations * 2 * Math.PI;
  }
  public double radiansToRotations(double radians){
    return radians/(2 * Math.PI);
  }

  public double getRadians(){
    return rotationToRadians(getPosition());
  }
  public double rotationToDegrees(double rotations){
    return rotations * 360;
  }
  public double degreesToRotation(double degrees){
    return degrees/360;
  }
  public void setPowerUp(double power){
    dutyCycleOut.Output = -power;
    armMotor.setControl(dutyCycleOut);
  }
  public void setPower(double power){
    dutyCycleOut.Output = power;
    dutyCycleOut.EnableFOC = true;
    armMotor.setControl(dutyCycleOut);
  }
  public void coast(){
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void brake(){
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public double getPosition(){
    return encoder.getAbsolutePosition() - encoder.getPositionOffset();
  }
  public void zeroEncoder(){
    encoder.reset();
  }
  //code needs to be fixed
  //find intakeHeight 
  public double calculateAngle(Pose3d estimatedRobotPose){
      double angle 
        = Math.tan((FieldConstants.speakerPose.getY() - estimatedRobotPose.getY()) / (estimatedRobotPose.getX() - FieldConstants.speakerPose.getX()));
      return this.radiansToRotations(angle);
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm position", getPosition());
    SmartDashboard.putNumber("Arm in Degrees", rotationToDegrees(getPosition()));
  }
}
