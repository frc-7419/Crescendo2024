// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.DeviceIDs.CanIds;

public class Wrist extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private TalonFX armMotor;
  private VoltageOut voltageOut;
  private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
  Velocity<Angle> RotationsPerMinute = Rotations.per(Minute);

  public Wrist() {
    armMotor = new TalonFX(CanIds.shooterWrist.id, "Ryan Biggee");
    voltageOut = new VoltageOut(12);
    armMotor.setInverted(true);
    armMotor.setPosition(0);

    // 0.587 should be a constant
    encoder.setPositionOffset(0.587 - ArmConstants.armOffset.in(Rotations));
  }

  public void coast() {
    armMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void brake() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setPower(double voltage) {
    voltageOut.Output = voltage;
    voltageOut.EnableFOC = true;
    armMotor.setControl(voltageOut);

  }

  public void setMMConfigs() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V outpu
    // slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    armMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void applyRequest(double position) {
    final MotionMagicVoltage armRequest = new MotionMagicVoltage(position, true, 0.0, 0, false, false, false);
    armMotor.setControl(armRequest);
  }

  public Measure<Angle> getPosition() {
    return Rotations.of(encoder.getAbsolutePosition() - encoder.getPositionOffset());
  }

  public double getPositionInRadians() {
    return getPosition().in(Radians);
  }

  public double getPositionInDegrees() {
    return getPosition().in(Degrees);
  }

  public Measure<Velocity<Angle>> getVelocity() {
    double velocity = armMotor.getVelocity().getValueAsDouble() / ArmConstants.armGearing;
    return RotationsPerSecond.of(velocity);
  }

  public double getVelocityInRadians() {
    return getVelocity().in(RadiansPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getPosition().magnitude());
    SmartDashboard.putNumber("Arm in Degrees", getPositionInDegrees());
  }
}
