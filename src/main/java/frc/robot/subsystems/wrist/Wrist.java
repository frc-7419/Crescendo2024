// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;
//TODO: add setpoints

public class Wrist extends SubsystemBase {
  private CANSparkMax wristMotor;
  public Wrist() {
    wristMotor = new CANSparkMax(CanIds.wrist.id, MotorType.kBrushless);
  }
  
  public void setSpeed(double speed){
    wristMotor.set(speed);
  }

  public void setVoltage(double voltage){
    wristMotor.setVoltage(voltage);
  }

  public void brake(){
    wristMotor.setIdleMode(IdleMode.kBrake);
  }

  public void coast(){
    wristMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
