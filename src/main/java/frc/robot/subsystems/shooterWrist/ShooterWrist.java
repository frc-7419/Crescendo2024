// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooterWrist;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class ShooterWrist extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX armMotor;
  private DutyCycleOut dutyCycleOut;
  public ShooterWrist() {
    armMotor = new TalonFX(CanIds.shooterWrist.id, "Ryan Biggee");
    dutyCycleOut = new DutyCycleOut(0);
    armMotor.setInverted(true);
  }

  public void setPower(double power){
    dutyCycleOut.Output = power;
    armMotor.setControl(dutyCycleOut);
  }
  public void setSetpoint(double setpoint){
    armMotor.setPosition(setpoint);
  }
  public void coast(){
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  public void brake(){
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  public double getPosition(){
    return armMotor.getPosition().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm position", this.getPosition());
  }
}
