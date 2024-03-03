// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax leftIntakeMotor;
  private CANSparkMax rightIntakeMotor;
  private CANSparkMax serializerBack;
  
  public IntakeSubsystem() {
    leftIntakeMotor = new CANSparkMax(CanIds.leftIntakeMotor.id, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(CanIds.rightIntakeMotor.id, MotorType.kBrushless);

    serializerBack = new CANSparkMax(CanIds.serializerBack.id, MotorType.kBrushless);
    invertMotors();
  }

  public void invertMotors(){
    serializerBack.setInverted(false);
    leftIntakeMotor.setInverted(true);
    rightIntakeMotor.setInverted(false);
  }
  
  //add voltage compensation and trapezoidal motion later
  public void setSpeed(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }

  public void setVoltage(double voltage) {
    leftIntakeMotor.setVoltage(voltage);
    rightIntakeMotor.setVoltage(voltage);
  }

  public void setSerializerVoltage(double voltage) {
    // serializerFront.setVoltage(voltage);
    serializerBack.setVoltage(voltage);
  }

  public void setSerializerSpeed(double speed) {
    // serializerFront.set(speed);
    serializerBack.set(speed);
  }

  public void brake(){
    leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    rightIntakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void brakeSerializer() {
    // serializerFront.setIdleMode(IdleMode.kBrake);
    serializerBack.setIdleMode(IdleMode.kBrake);
  }

  public void coast(){
    leftIntakeMotor.setIdleMode(IdleMode.kCoast);
    rightIntakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void coastSerializer() {
    // serializerFront.setIdleMode(IdleMode.kCoast);
    serializerBack.setIdleMode(IdleMode.kCoast);
  }

  public double getVoltage() {
    return leftIntakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("LeftIntakeSpeed", leftIntakeMotor.get());
      SmartDashboard.putNumber("RightIntakeSpeed", rightIntakeMotor.get());
      // SmartDashboard.putNumber("SerializerSpeed", serializerFront.get());
      SmartDashboard.putNumber("SerializerSpeed", serializerBack.get());
      SmartDashboard.putNumber("Current Draw", serializerBack.getOutputCurrent());
  }
}
