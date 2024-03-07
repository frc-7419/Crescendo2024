// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class Intake extends SubsystemBase {
  private CANSparkMax leftIntake;
  private CANSparkMax rightIntake;
  private CANSparkMax serializer;

  private DigitalInput beamBreakFront = new DigitalInput(1);
  private DigitalInput beamBreakBack = new DigitalInput(2);
  
  public Intake() {
    leftIntake = new CANSparkMax(CanIds.leftIntakeMotor.id, MotorType.kBrushless);
    rightIntake = new CANSparkMax(CanIds.rightIntakeMotor.id, MotorType.kBrushless);

    serializer = new CANSparkMax(CanIds.serializerBack.id, MotorType.kBrushless);
    invertMotors();
  }

  public boolean frontBeamBreakIsTriggered(){
    return !beamBreakFront.get();
  }
  public boolean backBeamBreakIsTriggered(){
    return !beamBreakBack.get();
  }

  public void invertMotors(){
    serializer.setInverted(false);
    leftIntake.setInverted(true);
    rightIntake.setInverted(false);
  }
  
  //add voltage compensation and trapezoidal motion later
  public void setSpeed(double speed) {
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

  public void setVoltage(double voltage) {
    leftIntake.setVoltage(voltage);
    rightIntake.setVoltage(voltage);
  }

  public void setSerializerVoltage(double voltage) {
    serializer.setVoltage(voltage);
  }

  public void setSerializerSpeed(double speed) {
    serializer.set(speed);
  }

  public void brake(){
    leftIntake.setIdleMode(IdleMode.kBrake);
    rightIntake.setIdleMode(IdleMode.kBrake);
  }

  public void brakeSerializer() {
    serializer.setIdleMode(IdleMode.kBrake);
  }

  public void coast(){
    leftIntake.setIdleMode(IdleMode.kCoast);
    rightIntake.setIdleMode(IdleMode.kCoast);
  }

  public void coastSerializer() {
    serializer.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("LeftIntakeSpeed", leftIntake.get());
      SmartDashboard.putNumber("RightIntakeSpeed", rightIntake.get());
      SmartDashboard.putNumber("SerializerSpeed", serializer.get());
      SmartDashboard.putNumber("Current Draw", serializer.getOutputCurrent());
      
      SmartDashboard.putBoolean("Has Note", frontBeamBreakIsTriggered());
  }
}
