// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.constants.DeviceIDs.CanIds;
//TODO: add setpoints
import frc.robot.constants.RobotConstants.WristConstants;

public class Wrist extends SubsystemBase {
  private CANSparkMax wristMotor;
  private TrapezoidProfile.Constraints constraints;

  public Wrist() {
    wristMotor = new CANSparkMax(CanIds.wrist.id, MotorType.kBrushless);
    constraints = 
      new TrapezoidProfile.Constraints(WristConstants.maxVelocity, WristConstants.maxAcceleration);
    zeroEncoder();
  }
  
  public void setSpeed(double speed){
    wristMotor.set(speed);
  }

  public void setVoltage(double voltage){
    wristMotor.setVoltage(voltage);
  }


  public double getVelocity() {
    return wristMotor.getEncoder().getVelocity();
  }

  public double getPosition() {
    return wristMotor.getEncoder().getPosition()*2*Math.PI;
  }

  public void brake(){
    wristMotor.setIdleMode(IdleMode.kBrake);
  }

  public void coast(){
    wristMotor.setIdleMode(IdleMode.kCoast);
  }

  public void zeroEncoder() {
    wristMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Constraints getConstraints() {
      return constraints;
  }
}
