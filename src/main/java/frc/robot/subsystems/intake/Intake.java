// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  
  public Intake() {
    intakeMotor = new CANSparkMax(CanIds.intakeMotor.id, MotorType.kBrushless);
  }
  //add voltage compensation and trapezoidal motion later
  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }
  public void setVoltage(double voltage){
    intakeMotor.setVoltage(voltage);
  }

  public void brake(){
    intakeMotor.setIdleMode(IdleMode.kBrake);
  }
  public void coast(){
    intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("IntakeSpeed", intakeMotor.get());
  }
}
