// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax intakeTurnMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(18, MotorType.kBrushless);
    intakeTurnMotor = new CANSparkMax(17, MotorType.kBrushless);
  }
  //add voltage compensation and trapezoidal motion later
  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }
  public void setTurnSpeed(double speed){
    intakeTurnMotor.set(speed);
  }
  // public void setIntakeDown(){
  //   intakeTurnMotor.
  // }
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
