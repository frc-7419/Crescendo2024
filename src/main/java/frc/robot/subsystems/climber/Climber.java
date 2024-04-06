// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs.CanIds;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climber;
  private final VoltageOut voltageRequest;
  private final CoastOut coastRequest;
  private final StaticBrake brakeRequest;
  private final SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();

  public Climber() {
    this.climber = new TalonFX(CanIds.climber.id,"Ryan Biggee" );
    configs.ForwardSoftLimitEnable = false;
    configs.ReverseSoftLimitEnable = false;

    configs.ForwardSoftLimitThreshold  = 30000;
    configs.ReverseSoftLimitThreshold = 500;
    this.climber.getConfigurator().apply(configs);

    voltageRequest = new VoltageOut(12);
    coastRequest = new CoastOut();
    brakeRequest = new StaticBrake();
  }

  public void setVoltage(double volts) {
    voltageRequest.Output = volts;
    voltageRequest.EnableFOC = true;
    climber.setControl(voltageRequest);
  }
  public void coast() {
    climber.setControl(coastRequest);
  }

  public void brake() {
    climber.setControl(brakeRequest);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
