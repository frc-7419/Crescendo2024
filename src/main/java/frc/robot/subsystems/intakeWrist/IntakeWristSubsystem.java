// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeWrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DeviceIDs.CanIds;
//TODO: add setpoints
import frc.robot.constants.RobotConstants.IntakeWristConstants;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class IntakeWristSubsystem extends SubsystemBase {
  private CANSparkMax wristMotor;
  private TrapezoidProfile.Constraints constraints;

  public IntakeWristSubsystem() {
    wristMotor = new CANSparkMax(CanIds.wrist.id, MotorType.kBrushless);
    constraints = IntakeWristConstants.constraints;
    zeroEncoder();
  } 
  // mogus
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine =
    new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        // Tell SysId how to plumb the driving voltage to the motors.
        (Measure<Voltage> volts) -> {
          wristMotor.setVoltage(volts.in(Volts));
        },
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        // characterized.
        log -> {
          // Record a frame for the left motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
          log.motor("swrist")
            .voltage(
              m_appliedVoltage.mut_replace(
                wristMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(wristMotor.getEncoder().getPosition(), Meters))
            .linearVelocity(
              m_velocity.mut_replace(wristMotor.getEncoder().getVelocity(), MetersPerSecond));
          // Record a frame for the right motors.  Since these share an encoder, we consider
          // the entire group to be one motor.
          // log.motor("shooter-top")
          //   .voltage(
          //     m_appliedVoltage.mut_replace(
          //       shooterMotorTop.get() * RobotController.getBatteryVoltage(), Volts))
          //   .linearPosition(m_distance.mut_replace(shooterMotorTop.getEncoder().getPosition(), Meters))
          //   .linearVelocity(
          //     m_velocity.mut_replace(shooterMotorTop.getEncoder().getVelocity(), MetersPerSecond));
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("drive")
        this));
  
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
    SmartDashboard.putNumber("Wrist position", getPosition());
  }
}
